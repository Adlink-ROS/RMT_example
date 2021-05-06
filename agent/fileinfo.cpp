#include <stdio.h>
#include <string.h>

int import_testfile(char *file_buf, int file_len) {
    printf("import test file.\n");
    return 1234;
}

int export_testfile(char *file_buf, int file_len) {
    printf("Modify export file\n");
    file_buf[2] = 'a';
    file_buf[3] = 't';
    return 2345;
}