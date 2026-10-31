#include "h264.h"
#include <string.h>
#include "esp_log.h"
#include "esp_h264_dec.h"
#include "esp_h264_dec_sw.h"
#include "esp_h264_types.h"
#include "esp_h264_dec_param.h"
#include "ps_diag.h"

static const char *TAG = "H264_DEC";

static esp_h264_dec_handle_t s_decoder = NULL;
static h264_frame_callback_t s_frame_callback = NULL;
static void *s_user_ctx = NULL;
static esp_h264_dec_param_handle_t s_param = NULL;
static h264_sps_info_t s_sps_info;          // geometry from the most recent SPS
static bool            s_sps_valid = false;

// ─── SPS parsing (picture size + cropping) ────────────────────────────────────

typedef struct {
    const uint8_t *p;
    int len, pos;      // pos in bits, over the NAL payload with 0x000003 removed
    int zeros;         // consecutive zero bytes seen (emulation prevention)
    int byte_idx;      // index of next raw byte
    uint32_t cur;      // current de-escaped byte
    int bits_left;
    bool err;
} bitreader_t;

static int br_next_byte(bitreader_t *b)
{
    while (b->byte_idx < b->len) {
        uint8_t c = b->p[b->byte_idx++];
        if (b->zeros >= 2 && c == 0x03) { b->zeros = 0; continue; }  // emulation prevention
        b->zeros = (c == 0) ? b->zeros + 1 : 0;
        return c;
    }
    return -1;
}

static uint32_t br_u(bitreader_t *b, int n)
{
    uint32_t v = 0;
    while (n-- > 0) {
        if (b->bits_left == 0) {
            int c = br_next_byte(b);
            if (c < 0) { b->err = true; return 0; }
            b->cur = (uint32_t)c;
            b->bits_left = 8;
        }
        v = (v << 1) | ((b->cur >> --b->bits_left) & 1);
    }
    return v;
}

static uint32_t br_ue(bitreader_t *b)
{
    int lz = 0;
    while (!b->err && br_u(b, 1) == 0) {
        if (++lz > 31) { b->err = true; return 0; }
    }
    if (lz == 0) return 0;
    return ((1u << lz) - 1) + br_u(b, lz);
}

static int32_t br_se(bitreader_t *b)
{
    uint32_t k = br_ue(b);
    return (k & 1) ? (int32_t)((k + 1) / 2) : -(int32_t)(k / 2);
}

static void br_skip_scaling_list(bitreader_t *b, int size)
{
    int last = 8, next = 8;
    for (int j = 0; j < size && !b->err; j++) {
        if (next != 0) next = (last + br_se(b) + 256) % 256;
        last = (next == 0) ? last : next;
    }
}

bool h264_parse_sps(const uint8_t *nal, int len, h264_sps_info_t *out)
{
    if (!nal || len < 4 || (nal[0] & 0x1F) != 7) return false;
    bitreader_t b = { .p = nal, .len = len, .byte_idx = 1 };  // skip NAL header
    uint32_t profile_idc = br_u(&b, 8);
    br_u(&b, 8);                    // constraint flags
    br_u(&b, 8);                    // level_idc
    br_ue(&b);                      // seq_parameter_set_id
    uint32_t chroma_format_idc = 1;
    if (profile_idc == 100 || profile_idc == 110 || profile_idc == 122 || profile_idc == 244 ||
        profile_idc == 44  || profile_idc == 83  || profile_idc == 86  || profile_idc == 118 ||
        profile_idc == 128 || profile_idc == 138 || profile_idc == 139 || profile_idc == 134 ||
        profile_idc == 135) {
        chroma_format_idc = br_ue(&b);
        if (chroma_format_idc == 3) br_u(&b, 1);   // separate_colour_plane_flag
        br_ue(&b);                                  // bit_depth_luma_minus8
        br_ue(&b);                                  // bit_depth_chroma_minus8
        br_u(&b, 1);                                // qpprime_y_zero_transform_bypass
        if (br_u(&b, 1)) {                          // seq_scaling_matrix_present
            int n = (chroma_format_idc == 3) ? 12 : 8;
            for (int i = 0; i < n && !b.err; i++)
                if (br_u(&b, 1)) br_skip_scaling_list(&b, i < 6 ? 16 : 64);
        }
    }
    br_ue(&b);                                      // log2_max_frame_num_minus4
    uint32_t poc_type = br_ue(&b);
    if (poc_type == 0) {
        br_ue(&b);                                  // log2_max_pic_order_cnt_lsb_minus4
    } else if (poc_type == 1) {
        br_u(&b, 1);
        br_se(&b); br_se(&b);
        uint32_t n = br_ue(&b);
        for (uint32_t i = 0; i < n && i < 256 && !b.err; i++) br_se(&b);
    }
    br_ue(&b);                                      // max_num_ref_frames
    br_u(&b, 1);                                    // gaps_in_frame_num_allowed
    uint32_t w_mbs = br_ue(&b) + 1;
    uint32_t h_map = br_ue(&b) + 1;
    uint32_t frame_mbs_only = br_u(&b, 1);
    if (!frame_mbs_only) br_u(&b, 1);               // mb_adaptive_frame_field
    br_u(&b, 1);                                    // direct_8x8_inference
    uint32_t cl = 0, cr = 0, ct = 0, cb = 0;
    if (br_u(&b, 1)) {                              // frame_cropping_flag
        cl = br_ue(&b); cr = br_ue(&b); ct = br_ue(&b); cb = br_ue(&b);
    }
    if (b.err) return false;

    // VUI colour description (optional; spec defaults: limited range,
    // unspecified matrix). Read errors here don't invalidate the geometry.
    bool full_range = false;
    uint8_t matrix = 2;
    if (br_u(&b, 1)) {                              // vui_parameters_present_flag
        if (br_u(&b, 1)) {                          // aspect_ratio_info_present_flag
            if (br_u(&b, 8) == 255) { br_u(&b, 16); br_u(&b, 16); }   // Extended_SAR
        }
        if (br_u(&b, 1)) br_u(&b, 1);               // overscan_info / overscan_appropriate
        if (br_u(&b, 1)) {                          // video_signal_type_present_flag
            br_u(&b, 3);                            // video_format
            uint32_t fr = br_u(&b, 1);              // video_full_range_flag
            uint32_t m = 2;
            if (br_u(&b, 1)) {                      // colour_description_present_flag
                br_u(&b, 8); br_u(&b, 8);           // colour_primaries, transfer
                m = br_u(&b, 8);                    // matrix_coefficients
            }
            if (!b.err) { full_range = fr; matrix = (uint8_t)m; }
        }
    }

    uint32_t coded_w = w_mbs * 16;
    uint32_t coded_h = h_map * 16 * (2 - frame_mbs_only);
    uint32_t cux = (chroma_format_idc == 1 || chroma_format_idc == 2) ? 2 : 1;
    uint32_t cuy = ((chroma_format_idc == 1) ? 2 : 1) * (2 - frame_mbs_only);
    if (chroma_format_idc == 0) { cux = 1; cuy = 2 - frame_mbs_only; }
    uint32_t x = cux * cl, y = cuy * ct;
    if (cux * (cl + cr) >= coded_w || cuy * (ct + cb) >= coded_h || coded_w > 4096 || coded_h > 4096)
        return false;
    out->coded_w = coded_w;
    out->coded_h = coded_h;
    out->x = x;
    out->y = y;
    out->width  = coded_w - cux * (cl + cr);
    out->height = coded_h - cuy * (ct + cb);
    out->full_range = full_range;
    out->matrix = matrix;
    return true;
}

// Remember the geometry of any SPS passing through the decoder.
static void note_sps(const uint8_t *data, uint32_t size)
{
    int sc = 0;
    if (size > 4 && data[0] == 0 && data[1] == 0 && data[2] == 0 && data[3] == 1) sc = 4;
    else if (size > 3 && data[0] == 0 && data[1] == 0 && data[2] == 1) sc = 3;
    if (!sc || (data[sc] & 0x1F) != 7) return;
    h264_sps_info_t info;
    if (h264_parse_sps(data + sc, size - sc, &info)) {
        if (!s_sps_valid || memcmp(&info, &s_sps_info, sizeof(info)) != 0)
            ESP_LOGI(TAG, "SPS: %ux%u visible (coded %ux%u, offset %u,%u), %s range, %s",
                     info.width, info.height, info.coded_w, info.coded_h, info.x, info.y,
                     info.full_range ? "full" : "limited",
                     info.matrix == 1 ? "BT.709" : (info.matrix == 2 ? "BT.601 (unspecified)" : "BT.601"));
        s_sps_info = info;
        s_sps_valid = true;
    }
}

esp_err_t h264_decoder_init(const h264_decoder_config_t *config)
{
    if (s_decoder != NULL) {
        ESP_LOGW(TAG, "Decoder already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Save callback
    s_frame_callback = config->on_frame_decoded;
    s_user_ctx = config->user_ctx;

    // Configure decoder for I420 output (YUV420 planar)
    esp_h264_dec_cfg_sw_t dec_cfg = {
        .pic_type = ESP_H264_RAW_FMT_I420,
    };

    // Create decoder
    esp_h264_err_t ret = esp_h264_dec_sw_new(&dec_cfg, &s_decoder);
    if (ret != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "Failed to create decoder: %d", ret);
        return ESP_FAIL;
    }

    // Open decoder
    ret = esp_h264_dec_open(s_decoder);
    if (ret != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "Failed to open decoder: %d", ret);
        esp_h264_dec_del(s_decoder);
        s_decoder = NULL;
        return ESP_FAIL;
    }

    // Parameter handle lets us read the real picture size from the SPS.
    if (esp_h264_dec_sw_get_param_hd(s_decoder, &s_param) != ESP_H264_ERR_OK) {
        ESP_LOGW(TAG, "No decoder param handle; frame size will be inferred");
        s_param = NULL;
    }

    ESP_LOGI(TAG, "H.264 decoder initialized");
    return ESP_OK;
}

esp_err_t h264_decoder_decode(const uint8_t *data, uint32_t size)
{
    if (s_decoder == NULL) {
        ESP_LOGE(TAG, "Decoder not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || size == 0) {
        ESP_LOGE(TAG, "Invalid input data");
        return ESP_ERR_INVALID_ARG;
    }

    note_sps(data, size);

    esp_h264_dec_in_frame_t in_frame = {
        .raw_data.buffer = (uint8_t *)data,
        .raw_data.len = size,
    };
    esp_h264_dec_out_frame_t out_frame = {0};

    // Process all NAL units in the buffer
    while (in_frame.raw_data.len > 0) {
        int64_t t0 = ps_now_us();
        esp_h264_err_t ret = esp_h264_dec_process(s_decoder, &in_frame, &out_frame);
        ps_perf_add(PS_STAGE_DECODE, t0);
        if (ret != ESP_H264_ERR_OK) {
            // Decode error - skip this data and wait for next keyframe
            // This is expected with UDP packet loss
            break;
        }

        // Update buffer pointer for next iteration
        in_frame.raw_data.buffer += in_frame.consume;
        in_frame.raw_data.len -= in_frame.consume;

        // If we got a decoded frame, invoke callback
        // The callback runs synchronously: outbuf is only valid until the
        // next esp_h264_dec_process() call, so it must not be held onto.
        if (out_frame.out_size > 0 && s_frame_callback != NULL) {
            h264_frame_t f = { .data = out_frame.outbuf, .size = out_frame.out_size };
            esp_h264_resolution_t res = {0};
            if (s_param && esp_h264_dec_get_resolution(s_param, &res) == ESP_H264_ERR_OK &&
                    (uint32_t)res.width * res.height * 3 / 2 == out_frame.out_size) {
                f.coded_w = res.width;
                f.coded_h = res.height;
            } else if (s_sps_valid &&
                    (uint32_t)s_sps_info.coded_w * s_sps_info.coded_h * 3 / 2 == out_frame.out_size) {
                f.coded_w = s_sps_info.coded_w;
                f.coded_h = s_sps_info.coded_h;
            }
            // Visible picture: SPS cropping, if it matches this buffer.
            if (f.coded_w && s_sps_valid &&
                    s_sps_info.coded_w == f.coded_w && s_sps_info.coded_h == f.coded_h) {
                f.x = s_sps_info.x;           f.y = s_sps_info.y;
                f.width = s_sps_info.width;   f.height = s_sps_info.height;
                f.full_range = s_sps_info.full_range;
                f.bt709 = (s_sps_info.matrix == 1);
            } else {
                f.width = f.coded_w;          f.height = f.coded_h;
            }
            // Sizes stay 0 if unknown; the callback infers them.
            s_frame_callback(&f, s_user_ctx);
        }
    }

    return ESP_OK;
}

esp_err_t h264_decoder_deinit(void)
{
    if (s_decoder == NULL) {
        ESP_LOGW(TAG, "Decoder not initialized");
        return ESP_OK;
    }

    esp_h264_dec_close(s_decoder);
    esp_h264_dec_del(s_decoder);
    s_decoder = NULL;
    s_param = NULL;
    s_sps_valid = false;
    s_frame_callback = NULL;
    s_user_ctx = NULL;

    ESP_LOGI(TAG, "H.264 decoder deinitialized");
    return ESP_OK;
}

bool h264_decoder_is_initialized(void)
{
    return s_decoder != NULL;
}
