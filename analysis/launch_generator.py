#!/usr/env python
# -*- coding: utf-8 -*-
from jinja2 import Template

data = {
	
}

f_in = open('testbed_template.launch', 'r')
template = Template(f_in.read())
output = template.render(data)
f_in.close()

f_out = open('testbed.launch', 'w')
f_out.write(output)
f_out.close()
