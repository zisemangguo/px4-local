#!/usr/bin/env python
from __future__ import print_function

import os
import sys
import subprocess
import re

filename = sys.argv[1]

try:
    fp_header = open(filename, 'r')
    old_header = fp_header.read()
except:
    old_header = ''


# Generate the header file content
header = """

/* Auto Magically Generated file */
/* Do not edit! */
#pragma once
#define PX4_GIT_VERSION_STR  "cbdb08bb6109c1ea3ef4e23761233df447523a52"
#define PX4_GIT_VERSION_BINARY 0xcbdb08bb6109c1ea
#define PX4_GIT_TAG_STR  "v1.7.0"
#define PX4_GIT_BRANCH_NAME  ""

#define NUTTX_GIT_VERSION_STR  "b18053574bf41712cef93e31bf178518f451a350"
#define NUTTX_GIT_VERSION_BINARY 0xb18053574bf41712
#define NUTTX_GIT_TAG_STR  "v7.22.0"

#define MAVLINK_LIB_GIT_VERSION_STR  "0667deeba1b156681a843ce40f77d1fd0d735513"
#define MAVLINK_LIB_GIT_VERSION_BINARY 0x0667deeba1b15668

"""

if old_header != header:
    print('Updating header {}'.format(sys.argv[1]))
    fp_header = open(filename, 'w')
    fp_header.write(header)
