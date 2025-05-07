# robot_cmd

<p align='right'>neozng1@hnu.edu.cn</p>

## 运行流程

运行流程可以很直观的从`RobotCMDTask()`中看出。

1. 首先通过消息订阅机制，获取其他应用的反馈信息
2. 使用`CalcOffsetAngle()`计算底盘和云台的offset angle（使得底盘始终获知当前的正方向）
3. 接着根据当前是通过键鼠or遥控器控制，调用对应的函数，将控制指令量化为具体的控制信息
4. 得到控制信息之后，先不急着发布，而是检测重要的模块和应用是否掉线或出现异常，以及遥控器是否进入紧急停止模式，如果以上情况发生，那么将所有的控制信息都置零，即让电机和其他执行单元保持静止。
5. 最后通过pubsub机制，把具体的控制信息发布到对应话题，让其他应用获取。若为双板，则将原本要推送给底盘的信息通过CANComm进行发送。



### 遥控器控制模式:

<table>
    <tr>
        <td align="center"><b>左侧开关<b></td> 
        <td align="center"><b>右侧开关<b></td> 
        <td align="center"><b>功能</b></td>
        <td align="center"><b>模式</b></td>
   </tr>
    <tr>
        <td align="center" rowspan=3>上</td>
  		<td align="center">上</td>
        <td align="center" rowspan=3>底盘云台控制</td>
      	<td align="center">正常行进模式</td>
    </tr>
    <tr>
        <td align="center">中</td> 
        <td align="center">取矿行进模式</td>
    </tr>
    <tr>
        <td align="center">下</td>
        <td align="center">紧急停止模式</td>   
    </tr>
    <tr>
        <td align="center" rowspan=3>中</td>
  		<td align="center">上</td>
        <td align="center" rowspan=3>上层机构控制</td>
      	<td align="center">滑移机构控制</td>
    </tr>
    <tr>
        <td align="center">中</td>
        <td align="center">机械臂控制</td>
    </tr>
    <tr>
        <td align="center">下</td>
        <td align="center">紧急停止模式</td>
    </tr>
    <tr>
        <td align="center" rowspan=3>下</td>    
  		<td align="center">上</td>
        <td align="center" rowspan=3>未定义</td>
      	<td align="center">未定义</td> 
    </tr>
    <tr>
        <td align="center">中</td> 
        <td align="center">未定义</td>    
    </tr>
    <tr>
        <td align="center">下</td> 
        <td align="center">紧急停止模式</td>    
    </tr>
    <tr>
        <td align="center" colspan=2><b>拨轮<b></td>
        <td align="center"><b>功能</b></td>
        <td align="center"><b>模式</b></td>
    </tr>
    <tr>
        <td align="center" colspan=2>上</td>    
        <td align="center" rowspan=2>气泵控制</td>
        <td align="center">开</td> 
    </tr>
    <tr>
        <td align="center" colspan=2>下</td>
        <td align="center">关</td> 
    </tr>
</table>

### 键鼠控制模式:

遥控器左侧开关拨到最上方,进入键鼠控制模式,此时不会响应遥控器遥感和拨轮的输入.

前后左右:WSAD

开关弹舱盖:R

小陀螺:Q

发射:鼠标左键

自瞄:鼠标右键







