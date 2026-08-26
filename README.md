# 说明

## 向上取整
```
#define KEY_DEBOUNCE_SCAN_COUNT                                      \
    ((KEY_INPUT_DEBOUNCE_TIME_MS + KEY_INPUT_SCAN_PERIOD_MS - 1U) / \
     KEY_INPUT_SCAN_PERIOD_MS)
```
这一段就是 KEY_INPUT_DEBOUNCE_TIME_MS 对 KEY_INPUT_SCAN_PERIOD_MS的向上取整