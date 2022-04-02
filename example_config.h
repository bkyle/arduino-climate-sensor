#pragma once

// Evaluates a value provided in seconds into a value in milliseconds.  For example
// `SECONDS(5)` would evaluate to `5000`.
#define SECONDS(x) (static_cast<int>(x * 1000))

// Pin used to call up the display.  Subsequently the status and sensor values
// will be displayed until CALLUP_TIMEOUT_MS have elapsed.
#define CALLUP_PIN 0

// SDA and SCL pins for the I2C bus that the display and sensor are attached to.
#define SDA_PIN 2
#define SCL_PIN 1


// If the device reboots within DRD_RESET_TIMEOUT_MS after boot then the
// device should be reset to default configuration.  A value of 0 disables
// this functionality.
#define DRD_RESET_TIMEOUT_MS      SECONDS(1)

// @see ESP_DoubleResetDetector.h
#define DOUBLERESETDETECTOR_DEBUG false

// @see ESP_DoubleResetDetector.h
#define ESP_DRD_USE_SPIFFS        true


// Number of seconds representing the interval at which sensor values should
// be consumed and reported.
#define UPDATE_INTERVAL_MS        SECONDS(5)

// Number of seconds the display should be active for after being called-up
// via the CALLUP_PIN.
#define CALLUP_TIMEOUT_MS         SECONDS(30)


// Flag indicating whether TLS should be used to connect to the MQTT broker.
// If set to true, MQTT_PORT and MQTT_CA_CERT must be set appropriately.
#define MQTT_USE_TLS       false

// Fully qualified host name of the MQTT broker that sensor events will be
// published to.
#define MQTT_BROKER        "public.mqtthq.com"

// Port that the MQTT broker is running on.  If the broker is running on
// standard ports then this will be 1883 for TCP and 8883 for TLS.
#define MQTT_PORT          1883

// Prefix of the topic that sensor events will be published to.  The full topic
// name consists of MQTT_TOPIC_PREFIX and the device identifier.
#define MQTT_TOPIC_PREFIX  "devices/"

// Username and password to use when connecting to the MQTT broker.  For MQTT
// brokers that support anonymous connections leave these as empty strings.
#define MQTT_USERNAME      ""
#define MQTT_PASSWORD      ""

// PEM-encoded certificate of the CA that signed the certificate for 
// the MQTT broker.  This certificate can be acquired by running the
// following command:
//
//     openssl s_client -connect [MQTT_BROKER]:[MQTT_PORT] -showcerts
//
// The CA certificate will typically be the last one listed.
#define MQTT_CA_CERT \
    "-----BEGIN CERTIFICATE-----\n" \
    "MIIFYDCCBEigAwIBAgIQQAF3ITfU6UK47naqPGQKtzANBgkqhkiG9w0BAQsFADA/\n" \
    "MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT\n" \
    "DkRTVCBSb290IENBIFgzMB4XDTIxMDEyMDE5MTQwM1oXDTI0MDkzMDE4MTQwM1ow\n" \
    "TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n" \
    "cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwggIiMA0GCSqGSIb3DQEB\n" \
    "AQUAA4ICDwAwggIKAoICAQCt6CRz9BQ385ueK1coHIe+3LffOJCMbjzmV6B493XC\n" \
    "ov71am72AE8o295ohmxEk7axY/0UEmu/H9LqMZshftEzPLpI9d1537O4/xLxIZpL\n" \
    "wYqGcWlKZmZsj348cL+tKSIG8+TA5oCu4kuPt5l+lAOf00eXfJlII1PoOK5PCm+D\n" \
    "LtFJV4yAdLbaL9A4jXsDcCEbdfIwPPqPrt3aY6vrFk/CjhFLfs8L6P+1dy70sntK\n" \
    "4EwSJQxwjQMpoOFTJOwT2e4ZvxCzSow/iaNhUd6shweU9GNx7C7ib1uYgeGJXDR5\n" \
    "bHbvO5BieebbpJovJsXQEOEO3tkQjhb7t/eo98flAgeYjzYIlefiN5YNNnWe+w5y\n" \
    "sR2bvAP5SQXYgd0FtCrWQemsAXaVCg/Y39W9Eh81LygXbNKYwagJZHduRze6zqxZ\n" \
    "Xmidf3LWicUGQSk+WT7dJvUkyRGnWqNMQB9GoZm1pzpRboY7nn1ypxIFeFntPlF4\n" \
    "FQsDj43QLwWyPntKHEtzBRL8xurgUBN8Q5N0s8p0544fAQjQMNRbcTa0B7rBMDBc\n" \
    "SLeCO5imfWCKoqMpgsy6vYMEG6KDA0Gh1gXxG8K28Kh8hjtGqEgqiNx2mna/H2ql\n" \
    "PRmP6zjzZN7IKw0KKP/32+IVQtQi0Cdd4Xn+GOdwiK1O5tmLOsbdJ1Fu/7xk9TND\n" \
    "TwIDAQABo4IBRjCCAUIwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMCAQYw\n" \
    "SwYIKwYBBQUHAQEEPzA9MDsGCCsGAQUFBzAChi9odHRwOi8vYXBwcy5pZGVudHJ1\n" \
    "c3QuY29tL3Jvb3RzL2RzdHJvb3RjYXgzLnA3YzAfBgNVHSMEGDAWgBTEp7Gkeyxx\n" \
    "+tvhS5B1/8QVYIWJEDBUBgNVHSAETTBLMAgGBmeBDAECATA/BgsrBgEEAYLfEwEB\n" \
    "ATAwMC4GCCsGAQUFBwIBFiJodHRwOi8vY3BzLnJvb3QteDEubGV0c2VuY3J5cHQu\n" \
    "b3JnMDwGA1UdHwQ1MDMwMaAvoC2GK2h0dHA6Ly9jcmwuaWRlbnRydXN0LmNvbS9E\n" \
    "U1RST09UQ0FYM0NSTC5jcmwwHQYDVR0OBBYEFHm0WeZ7tuXkAXOACIjIGlj26Ztu\n" \
    "MA0GCSqGSIb3DQEBCwUAA4IBAQAKcwBslm7/DlLQrt2M51oGrS+o44+/yQoDFVDC\n" \
    "5WxCu2+b9LRPwkSICHXM6webFGJueN7sJ7o5XPWioW5WlHAQU7G75K/QosMrAdSW\n" \
    "9MUgNTP52GE24HGNtLi1qoJFlcDyqSMo59ahy2cI2qBDLKobkx/J3vWraV0T9VuG\n" \
    "WCLKTVXkcGdtwlfFRjlBz4pYg1htmf5X6DYO8A4jqv2Il9DjXA6USbW1FzXSLr9O\n" \
    "he8Y4IWS6wY7bCkjCWDcRQJMEhg76fsO3txE+FiYruq9RUWhiF1myv4Q6W+CyBFC\n" \
    "Dfvp7OOGAN6dEOM4+qR9sdjoSYKEBpsr6GtPAQw4dy753ec5\n" \
    "-----END CERTIFICATE-----\n";
