#include "ota.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ota";

/* Enough of the file to read its app description: the image header, the
 * first segment header, then esp_app_desc_t. Held back and checked before
 * the first byte is written, so a wrong file never erases the slot. */
#define HEAD_LEN (sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + \
                  sizeof(esp_app_desc_t))

static bool s_active;
static bool s_began;                /* esp_ota_begin() done, handle is live */
static bool s_checked;
static bool s_pending;              /* running image is on trial */
static esp_ota_handle_t s_handle;
static const esp_partition_t *s_part;
static uint8_t s_head[HEAD_LEN];
static size_t s_head_len;
static size_t s_total, s_written;
static char s_err[96];
static char s_new_ver[33];

const char *ota_error(void)
{
    return s_err;
}

const char *ota_new_version(void)
{
    return s_new_ver;
}

bool ota_active(void)
{
    return s_active;
}

const char *ota_running_version(void)
{
    return esp_app_get_description()->version;
}

void ota_abort(void)
{
    if (s_began) {
        esp_ota_abort(s_handle);
        s_began = false;
    }
    s_active = false;
}

/* Records why, aborts, and returns ESP_FAIL — so every error path is one
 * line and none of them can leave a half-open handle behind. */
static esp_err_t fail(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_err, sizeof(s_err), fmt, ap);
    va_end(ap);
    ESP_LOGE(TAG, "%s", s_err);
    ota_abort();
    return ESP_FAIL;
}

esp_err_t ota_begin(size_t total)
{
    if (s_active) {
        snprintf(s_err, sizeof(s_err), "another firmware upload is running");
        return ESP_ERR_INVALID_STATE;
    }
    s_err[0] = '\0';
    s_new_ver[0] = '\0';

    s_part = esp_ota_get_next_update_partition(NULL);
    if (!s_part) {
        snprintf(s_err, sizeof(s_err),
                 "no OTA slot: flash the new partition table over USB once");
        return ESP_ERR_NOT_FOUND;
    }
    if (total < HEAD_LEN || total > s_part->size) {
        snprintf(s_err, sizeof(s_err), "size %u does not fit slot %s (%u bytes)",
                 (unsigned)total, s_part->label, (unsigned)s_part->size);
        return ESP_ERR_INVALID_SIZE;
    }

    s_total = total;
    s_written = 0;
    s_head_len = 0;
    s_checked = false;
    s_began = false;
    s_active = true;
    ESP_LOGW(TAG, "firmware upload: %u bytes into %s", (unsigned)total, s_part->label);
    return ESP_OK;
}

static esp_err_t check_head(void)
{
    const esp_image_header_t *h = (const esp_image_header_t *)s_head;
    if (h->magic != ESP_IMAGE_HEADER_MAGIC) {
        return fail("not an ESP32 firmware .bin");
    }
    if (h->chip_id != CONFIG_IDF_FIRMWARE_CHIP_ID) {
        return fail("firmware is for a different chip (id %d)", (int)h->chip_id);
    }

    const esp_app_desc_t *d = (const esp_app_desc_t *)
        (s_head + sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t));
    if (d->magic_word != ESP_APP_DESC_MAGIC_WORD) {
        return fail("no app description in the image");
    }
    const esp_app_desc_t *me = esp_app_get_description();
    if (strncmp(d->project_name, me->project_name, sizeof(d->project_name)) != 0) {
        return fail("this .bin is '%.31s', not '%.31s'", d->project_name, me->project_name);
    }

    snprintf(s_new_ver, sizeof(s_new_ver), "%.32s", d->version);
    ESP_LOGW(TAG, "image ok: %.32s version %.32s (running %.32s)",
             d->project_name, d->version, me->version);

    /* Sequential writes: the slot is erased sector by sector as data
     * arrives, instead of all 3 MB up front, which would stall the upload
     * for seconds. */
    if (esp_ota_begin(s_part, OTA_WITH_SEQUENTIAL_WRITES, &s_handle) != ESP_OK) {
        return fail("could not start writing slot %s", s_part->label);
    }
    s_began = true;
    s_checked = true;
    if (esp_ota_write(s_handle, s_head, HEAD_LEN) != ESP_OK) {
        return fail("flash write failed at the start");
    }
    s_written = HEAD_LEN;
    return ESP_OK;
}

esp_err_t ota_write(const uint8_t *data, size_t len)
{
    if (!s_active) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_checked) {
        size_t take = HEAD_LEN - s_head_len;
        if (take > len) {
            take = len;
        }
        memcpy(s_head + s_head_len, data, take);
        s_head_len += take;
        data += take;
        len -= take;
        if (s_head_len < HEAD_LEN) {
            return ESP_OK;
        }
        if (check_head() != ESP_OK) {
            return ESP_FAIL;
        }
    }
    if (len == 0) {
        return ESP_OK;
    }
    if (s_written + len > s_total) {
        return fail("more data than announced");
    }
    if (esp_ota_write(s_handle, data, len) != ESP_OK) {
        return fail("flash write failed at byte %u", (unsigned)s_written);
    }
    s_written += len;
    return ESP_OK;
}

esp_err_t ota_finish(void)
{
    if (!s_active) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_checked) {
        return fail("file too short to be firmware");
    }
    if (s_written != s_total) {
        return fail("got %u of %u bytes", (unsigned)s_written, (unsigned)s_total);
    }

    /* esp_ota_end() frees the handle whatever it returns. */
    esp_err_t err = esp_ota_end(s_handle);
    s_began = false;
    if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
        return fail("image check failed: the upload is corrupt");
    }
    if (err != ESP_OK) {
        return fail("finishing the image failed: %s", esp_err_to_name(err));
    }
    err = esp_ota_set_boot_partition(s_part);
    if (err != ESP_OK) {
        return fail("could not switch the boot slot: %s", esp_err_to_name(err));
    }

    s_active = false;
    ESP_LOGW(TAG, "firmware %s written to %s — reboot to run it", s_new_ver, s_part->label);
    return ESP_OK;
}

static void restart_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1500));    /* let the HTTP reply leave */
    ESP_LOGW(TAG, "rebooting into the new firmware");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

void ota_restart_soon(void)
{
    xTaskCreate(restart_task, "ota_reboot", 3072, NULL, 5, NULL);
}

void ota_boot_report(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_app_desc_t *me = esp_app_get_description();
    esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
    if (run) {
        esp_ota_get_state_partition(run, &st);
    }
    s_pending = st == ESP_OTA_IMG_PENDING_VERIFY;
    ESP_LOGI(TAG, "running %s, version %s, built %s %s%s",
             run ? run->label : "?", me->version, me->date, me->time,
             s_pending ? " — NEW, on trial for 30 s" : "");

    /* An update that did not survive its trial. Said loudly, because the
     * board otherwise looks fine — it is just running the old firmware. */
    const esp_partition_t *bad = esp_ota_get_last_invalid_partition();
    esp_app_desc_t bad_desc;
    if (bad && esp_ota_get_partition_description(bad, &bad_desc) == ESP_OK) {
        ESP_LOGW(TAG, "last update (%s, version %s) failed its trial and was rolled back",
                 bad->label, bad_desc.version);
    }
}

void ota_mark_good(void)
{
    if (!s_pending) {
        return;
    }
    s_pending = false;
    if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
        ESP_LOGW(TAG, "new firmware %s confirmed — no rollback",
                 esp_app_get_description()->version);
    }
}
