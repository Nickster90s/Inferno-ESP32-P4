// Web page on port 80: status, output volume and firmware update.
//
// The page (web_page.html) is served by the DAC8 but runs in the browser, and
// the browser is what talks to the update server: the DAC8 sits on the
// link-local Dante network with no route out, while the computer showing the
// page usually has one. The page asks the server for the latest firmware
// (that request is also the DAC8's check-in on the server), downloads it and
// posts it here; web.c writes it to the spare app slot, checks its signature
// against fw_sign_pub.pem and boots it.

#ifndef WEB_H
#define WEB_H

#include "esp_err.h"

esp_err_t web_start(void);

#endif // WEB_H
