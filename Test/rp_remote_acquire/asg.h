#ifndef ASG_H
#define ASG_H

#include "options.h"

struct asg_parameter {
    int     fd;
    void    *mapped_io;
    void    *mapped_buf_a;
    size_t  buf_a_size;
};

int asg_init(struct asg_parameter *param, option_fields_t *options);
void asg_cleanup(struct asg_parameter *param, option_fields_t *options);

#endif
