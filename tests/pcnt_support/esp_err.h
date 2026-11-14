#pragma once
#include <stdexcept>
typedef int esp_err_t;
enum { ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_STATE, ESP_ERR_NOT_FOUND, ESP_FAIL };
#define ESP_ERROR_CHECK(expression) do { if ((expression) != ESP_OK) throw std::runtime_error("ESP_ERROR_CHECK"); } while (0)
