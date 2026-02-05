#include <stdint.h>

<<<<<<< Updated upstream
int32_t cmd_vcpcert(int32_t argc, int8_t *argv[], int8_t **envp);
int32_t cmd_vcphsm(int32_t argc, int8_t *argv[], int8_t **envp);
int32_t cmd_vcpmcu(int32_t argc, int8_t *argv[], int8_t **envp);
=======
int32_t cmd_vcpmcu(int32_t argc, int8_t *argv[], int8_t **envp);
int32_t cmd_vcpcert(int32_t argc, int8_t *argv[], int8_t **envp);
int32_t cmd_vcphsm(int32_t argc, int8_t *argv[], int8_t **envp);
>>>>>>> Stashed changes

#define CMD_DESCS \
	struct cmd_desc { \
		const char *name; \
		int32_t (*fnc)(int32_t, int8_t **, int8_t **); \
	}; \
	\
	static struct cmd_desc cmds[] = { \
<<<<<<< Updated upstream
		{ .name = "vcpcert", .fnc = cmd_vcpcert, }, \
		{ .name = "vcphsm", .fnc = cmd_vcphsm, }, \
		{ .name = "vcpmcu", .fnc = cmd_vcpmcu, }, \
=======
		{ .name = "vcpmcu", .fnc = cmd_vcpmcu, }, \
		{ .name = "vcpcert", .fnc = cmd_vcpcert, }, \
		{ .name = "vcphsm", .fnc = cmd_vcphsm, }, \
>>>>>>> Stashed changes
	};

#define CMD_NUM (sizeof(cmds)/sizeof(struct cmd_desc))

