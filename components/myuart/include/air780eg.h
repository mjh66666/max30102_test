/*
 * @Author: mojionghao
 * @Date: 2024-08-27 19:46:26
 * @LastEditors: mojionghao
 * @LastEditTime: 2024-09-02 20:55:30
 * @FilePath: \max30102_test\components\myuart\include\air780eg.h
 * @Description:
 */
#pragma once

#define REV_OK 0   // 接收完成标志
#define REV_WAIT 1 // 接收未完成标志

#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>

typedef struct {
	uart_port_t bus;
	gpio_num_t restart_pin;
} air780eg_dev_t;

typedef void *air780eg_handle_t;

#define AT_EMQX_IP "AT+MIPSTART=\"8.134.252.177\",\"1883\"\r\n"   // EMQX地址
#define AT_EMQX_Client "AT+MCONFIG=\"19\",\"lbd\",\"lbd123\"\r\n" // 客户端ID及地址
#define AT_EMQX_CONNECT "AT+MCONNECT=1,360\r\n"                   // 客户端向服务器请求会话连接,具体参数见AT手册
#define AT_EMQX_PUB "AT+MPUB=\"test\",1,0,\"666\"\r\n"            // 客户端向服务器请求会话连接,具体参数见AT手册

enum DataType { // 枚举数据类型
	Int,
	Float,
	String
};

typedef struct {
	char msg[30];//消息名
	void *value; // 参数值
	enum DataType type;
}Mqtt_Data;

typedef struct{
    char* topic; //消息主题
    int qos;    //消息质量
    int retain; //保留标志
} Mqtt_config;

typedef struct{
    char* topic; //消息主题
    int qos;    //消息质量
    int retain; //保留标志
} Mqtt_config;

typedef struct 
{
	float longitude;
	float latitude;
} GNSS_data;
