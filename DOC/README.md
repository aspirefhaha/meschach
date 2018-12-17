# Introduction

```SHELL
# in vim
% s/\(\d\+,\d\+,\d\+\.\d\+,\d\+,\d\+\.\d\+\)/\t{\1},/g
```

* Update Control Gain C array script

```SHELL
% s/\(\d\+\.\?\d*,\d\+\.\?\d*,\d\+\.\?\d*,\d\+\.\?\d*,\d\+\.\?\d*\)/\t{\1},/g
```

* Update IMU Data C array script
```SHELL
## single data replace % s/\(-\?\d\+[\.]\?\d*E\?-\?\d\+\)/\t{\1},/g
% s/\(-\?\d\+\.\?\d*E\?-\?\d*,-\?\d\+\.\?\d*E\?-\?\d*,-\?\d\+\.\?\d*E\?-\?\d*,-
\?\d\+\.\?\d*E\?-\?\d*,-\?\d\+\.\?\d*E\?-\?\d*,-\?\d\+\.\?\d*E\?-\?\d*,-\?\d\+\.
\?\d*E\?-\?\d*\)/\t{\1},/g
```

* update trajectoryBZ and Height script
```SHELL
sed  -r 's/(^.*$)/{\1},/g' xxx.csv  > xxx.c
```
