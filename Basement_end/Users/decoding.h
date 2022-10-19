#ifndef __DECODING_H
#define __DECODING_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

//节点
typedef struct Command{
	char target_command[20];//存储命令，例如：a#b
	struct Command* next;
}Command;

//链表
typedef struct {
	Command* head;//头结点
	Command* tail;//尾节点
}List;

//a#b形式的命令参数
typedef struct a_b {
	int a;
	int b;
}a_b;

//c@d#e形式的命令参数
typedef struct c_d_e {
	int c;
	int d;
	int e;
}c_d_e;

//命令参数共用体，选择命令参数
typedef union Operate{
	a_b config_a_b;
	c_d_e config_c_d_e;
}Operate;

//命令配置，确定使用那种命令参数
typedef struct {
	int flag;//0: a#b  1: c@d#e -1 无效命令
	Operate operate;//共用体，确定使用a_b型或c_d_e型
}CommandConfig;


//链表初始化
void list_init(List* list);

//链表尾插
void list_append(List* list, Command* com);

//链表头删
void list_delete(List* list);

/*
参数：上位机发送的字符串
返回带有命令参数的链表，结构体内容为含有命令参数的字符数组
*/
/*####################################################*/
List* regex_string(const char* str);

/*
参数：命令字符串
返回值：特殊字符数量，例：a#b 中有一个特殊字符，c@d#e 中有三个特殊字符
*/
int speical_chara_num(const char* str);

/*
参数：链表（链表会自动共删除）
返回值：处理链表中的命令，并返回命令参数
*/
/*####################################################*/
CommandConfig deal_command(List* list);

#endif