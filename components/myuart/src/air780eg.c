/*
 * @Author: mojionghao
 * @Date: 2024-08-26 20:12:05
 * @LastEditors: mojionghao
 * @LastEditTime: 2024-09-03 16:44:38
 * @FilePath: \max30102_test\components\myuart\src\air780eg.c
 * @Description:
 */

#include "esp_log.h"
#include "air780eg.h"
#include "string.h"
#include "cJSON.h"

static const char *air780eg = "air780eg";

unsigned char Air780EG_buf[512]; // 串口缓冲区

int Air780EG_size = 0, Air780EG_lastsize = 0; // 接收数据计数值,上一个计数值

/**
 * @description: 串口初始化
 * @param {uart_port_t} num 位号
 * @param {int} rx
 * @param {int} tx
 * @param {int} rts 不使用的管脚指定为UART_PIN_NO_CHANGE
 * @param {int} cts
 * @return {*}
 */
static void Uart_Bus_Init(uart_port_t num, int rx, int tx, int rts, int cts)
{
	uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS, // 8位数据
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,         // 一位停止位
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // 禁用硬件流控
		.rx_flow_ctrl_thresh = 122,
	};
	// Configure UART parameters
	ESP_ERROR_CHECK(uart_param_config(num, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(num, rx, tx, rts, cts));
	const int uart_buffer_size = (1024 * 2);
	QueueSetHandle_t uart_queue;
	ESP_ERROR_CHECK(uart_driver_install(num, uart_buffer_size,
	                                    uart_buffer_size, 10, &uart_queue, 0));
}

/**
 * @description: 创建一个air780eg设备
 * @param {uart_port_t} num
 * @param {gpio_port_t} pin
 * @param {  } device
 * @return {*}
 */
air780eg_handle_t Air780EG_Create(uart_port_t num, gpio_port_t pin)
{
	air780eg_dev_t *device = (air780eg_dev_t *)calloc(1, sizeof(air780eg_dev_t)); // 分配内存
	device->bus = num;
	device->restart_pin = pin;
	return (air780eg_handle_t)device;
}

/**
 * @description: 删除air780eg设备
 * @param {air780eg_handle_t} dev
 * @return {*}
 */
void Air780eg_Delete(air780eg_handle_t dev)
{
	air780eg_dev_t *device = (air780eg_dev_t *)dev;
	free(device);
}

/**
 * @description:
 * @param {air780eg_handle_t} dev 设备句柄
 * @param {char} *cmd 输入的命令
 * @param {char} *ret 需要检查的返回指令，如“+xxx”，“ok”
 * @param {char} *respond 保存返回指令，读取参数
 * @return {*}
 */
esp_err_t Air780EG_Sendcmd(air780eg_handle_t dev, char *cmd, char *ret, char *respond)
{
	air780eg_dev_t *device = (air780eg_dev_t *)dev;
	uint8_t timewait = 200; // 等待次数 0~255
	int length = 0, prevDataLength = 0;
	unsigned char Air780EG_buf[256];
	uart_write_bytes(device->bus, (const char *)cmd, strlen(cmd)); // 发送命令
	while (timewait--) {                                           // 循环等待
		uart_get_buffered_data_len(device->bus, (size_t *)&length); // 查看缓冲区是否有数据
		if (length == prevDataLength || length != 0) {              // 接收完毕
			uart_read_bytes(device->bus, Air780EG_buf, length, 100);

			if (strstr((const char *)Air780EG_buf, ret) != NULL) {
				if (respond != NULL) {
					strcpy(respond, (const char *)Air780EG_buf);
				}
				uart_flush(device->bus);
				return ESP_OK;
			}
		}
		prevDataLength = length;
		vTaskDelay(10 / portTICK_PERIOD_MS); // 10ms
	}
	ESP_LOGE(air780eg, "send cmd: %s time out!",cmd);
	return ESP_FAIL;
}

esp_err_t Air780EG_Init(air780eg_handle_t dev)
{
	esp_err_t status = ESP_OK;

	air780eg_dev_t *device = (air780eg_dev_t *)dev;
	gpio_config_t Air780EG_Resart_conf = {};
	Air780EG_Resart_conf.intr_type = GPIO_INTR_DISABLE;
	Air780EG_Resart_conf.mode = GPIO_MODE_OUTPUT;
	Air780EG_Resart_conf.pin_bit_mask = 1 << device->restart_pin;
	status = gpio_config(&Air780EG_Resart_conf);

	gpio_set_level(device->restart_pin, 0);
	vTaskDelay(2000 / portTICK_PERIOD_MS);
	gpio_set_level(device->restart_pin, 1);
	vTaskDelay(2000 / portTICK_PERIOD_MS);

	ESP_LOGI(air780eg, "RESET!");

	uart_flush(device->bus);

	const char *commands[] = {
		"AT\r\n",
		"AT+CPIN?\r\n",
		"AT+CGATT?\r\n",
		AT_EMQX_IP,
		AT_EMQX_Client,
		AT_EMQX_CONNECT
	};
	const char *expectedResponses[] = {
		"OK\r\n",
		"READY",
		"+CGATT: 1",
		"CONNECT OK",
		"OK",
		"CONNACK OK"
	};

	for (int i = 0; i < 6; i++) {
		status = Air780EG_Sendcmd(dev, commands[i], expectedResponses[i], NULL);
		if (status != ESP_OK) {
			return status;
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	return status;
}

/**
 * @description: 字符转义
 * @param {char} *output
 * @param {char} *input
 * @return {*}
 */
unsigned char escape_json(char *output, const char *input)
{
	char *ptr = output; //指向缓冲区头
	while (*input) { // '\0'为0，循环结束
		switch (*input) {
			case '"':
				ptr += sprintf(ptr, "\\22"); //格式化 " -> \\22
				break;
			case '\r':
				ptr += sprintf(ptr, "\\0D");
				break;
			case '\n':
				ptr += sprintf(ptr, "\\0A");
				break;
			case '\\':
				ptr += sprintf(ptr, "\\5C");
				break;
			default:
				// 对于不需要转义的字符，直接拷贝到输出缓冲区
				*ptr = *input;
				ptr++;
				break;
		}
		input++;
	}
	*ptr = '\0'; // 结束字符串
	return (unsigned char)(ptr - output);
}

/**
 * @description: 
 * @param {char} *json_output 输出
 * @param {Data} data_array 消息数组
 * @param {int} count 数组个数
 * @return {unsigned char} len json长度
 */
unsigned char Air780EG_SavejsonData(char *json_output, Mqtt_Data data_array[], int count)
{
	//创建
	cJSON *json_obj = cJSON_CreateObject();
	if (json_obj == NULL) {
		ESP_LOGE(air780eg, "json object create fail!");
		return 0;
	}

	//填充
	for (int i = 0; i < count; i++) {
		switch (data_array[i].type) {
			case Int:
				cJSON_AddNumberToObject(json_obj, data_array[i].msg, *((int *)data_array[i].value));
				break;
			case Float:
				cJSON_AddNumberToObject(json_obj, data_array[i].msg, *((float *)data_array[i].value));
				break;
			case String:
				cJSON_AddStringToObject(json_obj, data_array[i].msg, *((char *)data_array[i].value));
			default:
				// 处理未知数据类型的情况
				break;
		}
	}

	//生成并转义
	char *json_str = cJSON_Print(json_obj);
	if (json_str == NULL) {
		cJSON_Delete(json_obj);
		ESP_LOGE(air780eg, "json object print fail!");
		return 0;
	}
	unsigned char len = escape_json(json_output, json_str);

	//释放内存
	free(json_str);
	cJSON_Delete(json_obj);

	return len;
}

//典型:AT+MPUB="test2",0,0,"{\22msg\22:\22say\22}"
/**
 * @description: 
 * @param {air780eg_handle_t} dev 设备 
 * @param {Mqtt_config} cfg 配置
 * @param {Data} data_array 消息数组
 * @param {int} count 消息数
 * @param {char} *respon 返回 无填null
 * @return {*}
 */
esp_err_t Air780EG_SendMqttData(air780eg_handle_t dev, Mqtt_config cfg,Mqtt_Data data_array[], int count, char *respond)
{
	esp_err_t status = ESP_OK;
	air780eg_dev_t *device = (air780eg_dev_t *)dev;
	char json_buf[150];
	char send_buf[200];

	memset(json_buf, 0, sizeof(json_buf));
	memset(send_buf, 0, sizeof(send_buf));

	int len = Air780EG_SavejsonData(json_buf, data_array, count);
	if (len == 0) {
		ESP_LOGE(air780eg, "Failed to save JSON data");
		return ESP_FAIL;
	}
	snprintf(send_buf, sizeof(send_buf), "AT+MPUB=\"%s\",%d,%d,\"%s\"\r\n", cfg.topic, cfg.qos, cfg.retain, json_buf);

	status = Air780EG_Sendcmd(dev, send_buf, "OK\r\n", respond);
	if (status == ESP_FAIL) {
		ESP_LOGE(air780eg, "Send data fail!");
		return status;
	}
	return status;
}

esp_err_t Air780EG_GNSSInit(air780eg_handle_t dev)
{
	esp_err_t status;
	char ret;
	const char *commands[] = {
		"AT+CGNSPWR=1\r\n",
		"AT+CGNSAID=31,1,1,1\r\n"
	};
	const char *expectedResponses[] = {
		"OK\r\n",
		"OK\r\n",
	};
	for (int i = 0; i < 2; i++) {
		status = Air780EG_Sendcmd(dev, commands[i],expectedResponses[i], ret);
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	return status;
}

esp_err_t Air780EG_GetGNSSData(air780eg_handle_t dev,GNSS_data* gps_data)
{
	esp_err_t status;
	char GNSS_respond[256];
    char *GNSS_position;
    int count = 0; // 计数值和定位状态
    float data[3]; // 经纬度存储

	//接收
	status = Air780EG_Sendcmd(dev,"AT+CGNSINF\r\n", "OK\r\n", GNSS_respond);
	if(status == ESP_FAIL)
	{
		ESP_LOGE(air780eg, "check position fail!");
		return 0;
	}
	GNSS_position = strtok(GNSS_respond, ","); // 分割字符串

	//提取信息
	while (GNSS_position != NULL) {

        if (count > 3 && count < 6) {              // 取4、5
            data[count - 3] = atof(GNSS_position); // 将相应的信息转换成浮点型之后存到数组里面
        }
        count++;
        GNSS_position = strtok(NULL, ",");
    }

	//返回
	if(gps_data != NULL)
	{
		gps_data->longitude = data[0];
		gps_data->latitude = data[1];
	}
	return status;
}

