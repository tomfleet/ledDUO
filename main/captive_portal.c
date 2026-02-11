#include "captive_portal.h"

#include "esp_http_server.h"
#include "config.h"

static esp_err_t http_404_handler(httpd_req_t *req, httpd_err_code_t err)
{
    (void)err;
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

esp_err_t captive_portal_start(captive_portal_t *portal,
                               esp_netif_t *ap_netif,
                               const web_dashboard_state_t *state)
{
#if CAPTIVE_PORTAL_ENABLE
    if (!portal || !ap_netif || !state) {
        return ESP_ERR_INVALID_ARG;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.uri_match_fn = httpd_uri_match_wildcard;

    esp_err_t err = httpd_start(&portal->server, &config);
    if (err != ESP_OK) {
        return err;
    }

    err = web_dashboard_init(state);
    if (err != ESP_OK) {
        return err;
    }

    web_dashboard_register_handlers(portal->server);
    httpd_register_err_handler(portal->server, HTTPD_404_NOT_FOUND, http_404_handler);
    return ESP_OK;
#else
    (void)portal;
    (void)ap_netif;
    (void)state;
    return ESP_OK;
#endif
}
