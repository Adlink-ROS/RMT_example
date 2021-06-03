#include <stdio.h>
#include <string.h>

int import_testfile(char *filename, char *file_buf, int file_len)
{
    printf("import filename: %s\n", filename);
    printf("import: length of file=%d\n", file_len);
    return file_len;
}

int export_testfile(char *filename, char *file_buf, int file_len)
{
    int ret = 0;

    printf("export filename: %s\n", filename);
    printf("Modify export file\n");
    if ((file_buf != NULL) && (file_len > 4)) {
        file_buf[2] = 'a';
        file_buf[3] = 't';
    } else {
        ret = -1;
    }

    return ret;
}
