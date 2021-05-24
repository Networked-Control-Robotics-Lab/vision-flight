#include <thread>
#include <termios.h>
#include "quadshell.hpp"
#include "shell_cmds.hpp"

void shell_cmd_help(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt);

struct cmd_list_entry shell_cmd_list[] = {
	DEF_SHELL_CMD(help)
	DEF_SHELL_CMD(clear)
	DEF_SHELL_CMD(exit)
	DEF_SHELL_CMD(quit)
	DEF_SHELL_CMD(mission)
};

void shell_greeting(void)
{
	char s[150];
	sprintf(s, "firmware build time: %s %s\n\rtype `help' for help\n\r\n\r", __TIME__, __DATE__);
	shell_puts(s);
}

void shell_thread_entry()
{
	system("/bin/stty raw -echo");

	/* init shell cli */
	struct shell_struct shell;
	shell_init_struct(&shell, "vafs > ");

	/* init shell parser */
	int shell_cmd_cnt = SIZE_OF_SHELL_CMD_LIST(shell_cmd_list);

	shell_cls();
	shell_greeting();

	while(1) {
		shell_cli(&shell);
		shell_cmd_exec(&shell, shell_cmd_list, shell_cmd_cnt);
	}
}

