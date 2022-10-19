#include "decoding.h"
#include <string.h>
void __null_op(char *p, ...)
{

}

#ifndef printf
#define printf __null_op
#endif
//链表初始化
void list_init(List* list) {
	list->head = NULL;
	list->tail = NULL;
}

//尾插
void list_append(List* list, Command* com) {

	if (list->head == NULL && list->tail == NULL) {
		list->head = com;
		list->tail = com;
	}
	else {
		list->tail->next = com;
		list->tail = com;
	}
}

//头删
void list_delete(List* list) {
	if (list->head == list->tail && list->head != NULL) {
		printf("del head and tail\n");
		free(list->head);
		list->head = NULL;
		list->tail = NULL;
	}
	else if (list->head == NULL && list->tail == NULL) {
		printf("null list\n");
	}
	else {
		printf("del\n");
		Command* p = list->head;
		list->head = p->next;
		free(p);
	}
}

/*
参数：上位机发送的字符串
返回带有命令参数的链表，结构体内容为含有命令参数的字符数组
*/
/*####################################################*/
List* regex_string(const char* str) 
{
	char* p;
	int i;
	List* list = (List*)malloc(sizeof(List));
	
	list_init(list);

	printf("%d\n", strlen(str));
	if (!((str[0] == '%') && (str[strlen(str) - 1] == '&'))) {
		return NULL;
	}
	else {

		p = (char*)str;

		for (i = 1; i < strlen(str) - 1; i++) {
			
			while (p[i] != ';') {
				Command* com = (Command*)malloc(sizeof(Command));
				com->next = NULL;
				memset(com->target_command, '\0', 20);
				for (int j = 0; j < 20; j++) {
					if (p[i] != ';') {
						com->target_command[j] = p[i];
						i++;
					}
					else {
						break;
					}
				}
				// for (int j = 0; j < 20; j++) {
				// 	printf("%c", com->target_command[j]);
				// }
				list_append(list, com);
			}
			printf("\n");
		}
	}
	
	return list;
}



/*
参数：命令字符串
返回值：特殊字符数量，例：a#b 中有一个特殊字符，c@d#e 中有三个特殊字符
*/
int speical_chara_num(const char* str) {
	const char* p = str;
	int count = 0;
	for (int i = 0; i < strlen(str) - 1; i++) {
		if (*(p+i) == '#' || *(p+i) == '@') {
			count++;
		}
	}
	return count;
}

/*
参数：链表（链表会自动共删除）
返回值：处理链表中的命令，并返回命令参数
*/
/*####################################################*/
CommandConfig deal_command(List* list) {
	Command* p = list->head;

	if (speical_chara_num(p->target_command) == 1) {
		
		CommandConfig config;
		config.flag = 0;

		int i = 0;
		int power = 1;
		int sum = 0;
		for (; i < 20 && p->target_command[i] != '\0'; i++) {
			if (p->target_command[i] == '#') {
				i++;
				break;
			}
			if (p->target_command[i] == '-') {
				power = -1;
			}
			if (p->target_command[i] >= 48 && p->target_command[i] <= 57) {
				sum = sum * 10 + p->target_command[i] - 48;
			}
		}
		sum *= power;

		config.operate.config_a_b.a = sum;
		printf("config a = %d\n", sum);

		power = 1;
		sum = 0;
		for (; i < 20 && p->target_command[i] != '\0'; i++) {
			if (p->target_command[i] == '#') {
				break;
			}
			/*if (p->target_command[i] == '-') {
				power = -1;
			}*/
			if (p->target_command[i] >= 48 && p->target_command[i] <= 57) {
				sum = sum * 10 + p->target_command[i] - 48;
			}
		}

		sum *= power;

		config.operate.config_a_b.b = sum;
		printf("config b = %d\n", sum);

		list_delete(list);
		return config;
	}
	else if (speical_chara_num(p->target_command) == 2) {
		CommandConfig config;
		config.flag = 1;

		int i, power, sum;

		i = 0;
		power = 1;
		sum = 0;
		for (; i < 20 && p->target_command[i] != '\0'; i++) {
			if (p->target_command[i] == '@' || p->target_command[i] == '#') {
				i++;
				break;
			}
			if (p->target_command[i] == '-') {
				power = -1;
			}
			if (p->target_command[i] >= 48 && p->target_command[i] <= 57) {
				sum = sum * 10 + p->target_command[i] - 48;
			}
		}
		sum *= power;

		config.operate.config_c_d_e.c = sum;
		printf("config c = %d\n", sum);

		power = 1;
		sum = 0;
		for (; i < 20 && p->target_command[i] != '\0'; i++) {
			if (p->target_command[i] == '@' || p->target_command[i] == '#') {
				i++;
				break;
			}
			if (p->target_command[i] == '-') {
				power = -1;
			}
			if (p->target_command[i] >= 48 && p->target_command[i] <= 57) {
				sum = sum * 10 + p->target_command[i] - 48;
			}
		}
		sum *= power;

		config.operate.config_c_d_e.d = sum;
		printf("config d = %d\n", sum);

		power = 1;
		sum = 0;
		for (; i < 20 && p->target_command[i] != '\0'; i++) {
			if (p->target_command[i] == '@' || p->target_command[i] == '#') {
				i++;
				break;
			}
			/*if (p->target_command[i] == '-') {
				power = -1;
			}*/
			if (p->target_command[i] >= 48 && p->target_command[i] <= 57) {
				sum = sum * 10 + p->target_command[i] - 48;
			}
		}
		sum *= power;

		config.operate.config_c_d_e.e = sum;
		printf("config e = %d\n", sum);

		list_delete(list);
		return config;
	}
	else {
		CommandConfig config;
		config.flag = -1;
		return config;
	}
}

#undef printf