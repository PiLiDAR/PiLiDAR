- [cn](#操作指南)
- [en](#Instructions)
# 操作指南

> 此SDK仅适用于深圳乐动机器人有限公司销售的激光雷达产品

## 1. 设备权限设置
- 可以通过`ls -l /dev/`查看雷达对应在系统中的挂载
> 例如：如果雷达在你的系统上挂载为`/dev/ttyS0`,则：
``` bash
sudo chmod 777 /dev/ttyS0
```

## 2. 编译

```bash
./auto_build.sh
```

## 3. 运行
``` bash
./start_node.sh
```

[回到仓库简介](../README.md)

# Instructions

> This SDK is only applicable to the LiDAR products sold by Shenzhen LDROBOT Co., LTD. 

## step 1: Device Permission Settings

- You can use `ls -l /dev/` to view the radar mounted on the system  
> For example, if the radar is mounted on your system as `/dev/ttys0`, then:  
``` bash
sudo chmod 777 /dev/ttyS0
```

## step 2: build

``` bash
./auto_build.sh
```

## step 3: run
``` bash
./start_node.sh
```

[ Back to the introduction ](../README.md)