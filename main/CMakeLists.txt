idf_component_register(
    SRCS
        "main.c"
        "rate.c"
        "eth_ts.c"
        "ptp_servo.c"
        "ptpv1.c"
        "media_clock.c"
        "mclk_hw.c"
        "jitterbuf.c"
        "audio_out.c"
        "pcm1690.c"
        "aoip_rx.c"
        "flows_client.c"
        "chan_resolve.c"
        "aoip_arc.c"
        "aoip_msg.c"
        "aoip_info.c"
        "reqlog.c"
        "aoip_mdns.c"
        "subscriber.c"
        "telem.c"
        "console.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_eth esp_netif esp_timer esp_driver_i2s esp_driver_spi esp_driver_i2c esp_driver_gpio
        lwip mdns nvs_flash esp_hw_support driver console vfs
)

# Sample rate.
#
# By default the rate is chosen at BOOT from AP_PIN_RATE_SEL:
#   open / high -> 48 kHz      closed to GND -> 96 kHz
#
# Passing SAMPLE_RATE pins it instead and the pin is ignored:
#   idf.py -DSAMPLE_RATE=96000 build
#
# Either way the same binary runs both rates -- buffers are sized for 96 kHz.
# The DAC needs no rewiring between them: SCKI is a fixed 24.576 MHz from a
# dedicated I2S port (512 fs at 48 kHz, 256 fs at 96 kHz are the same number),
# and only BCK changes.
if(DEFINED SAMPLE_RATE)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC AP_SAMPLE_RATE=${SAMPLE_RATE})
    message(STATUS "aoip-rx: sample rate PINNED at ${SAMPLE_RATE} Hz (rate pin ignored)")
else()
    message(STATUS "aoip-rx: sample rate selected at boot by AP_PIN_RATE_SEL")
endif()
