#remote_control
<p align='right'>neozng1@hnu.edu.cn</p>

```
/*************************发射机DT7***************************
 *                                                            *
 *   -----------------------------------------------------    *
 *   |     (上-1)                               (上-1)   |    *
 *   |SW_L|(中-3)                          SW_R|(中-3)   |    *
 *   |     (下-2)                               (下-2)   |    *
 *   |                                                   |    *
 *   |    | ^ |                                | ^ |     |    *
 *   |    | 3 |左摇杆                     右摇杆| 1 |     |    *
 *   | ---     ---                          ---     ---  |    *
 *   |<           2>                       <           0>|    *
 *   | ---     ---                          ---     ---  |    *
 *   |    |   |                                |   |     |    *
 *   |    |   |                                |   |     |    *
 *   |                                                   |    *
 *   -----------------------------------------------------    *
 *                                                            *
 **************************遥控器信息**************************
 *域            通道0      通道1   通道2   通道3   S1     S2  *
 *偏移            0          11      22      33    44     46  *
 *长度(bit)      11          11      11      11     2      2  *
 *符号位         无          无      无      无    无     无  *
 *范围           ***********最大值1684*********    *最大值3*  *
 *               *          中间值1024        *    *最小值1*  *
 *               ***********最小值364**********               *
 *功能                                            1:上   1:上 *
 *                                                2:下   2:下 *
 *                                                3:中   3:中 *
 *                                                            *
 ***************************鼠标信息***************************
 *域         鼠标x轴   鼠标y轴   鼠标z轴   鼠标左键   鼠标右键*
 *偏移         48        64        80        86        94   *
 *长度         16        16        16        8         8    *
 *符号位       有        有        有        无         无   *
 *范围         ******最大值32767*****        ***最大值1***   *
 *             *     最小值-32768   *        ***最小值0***   *
 *             ******静止值0*********                        *
 *功能       ***鼠标在XYZ轴的移动速度***   *鼠标左右键是否按下*
 *           *  负值表示往左移动       *   *    0:没按下      *
 *           ***正值表示往右移动********   *****1:按下*********
 *                                                           *
 *                                                           *
 ***************************键盘信息***************************
 *域              按键                                        *
 *偏移            102                                         *
 *长度            16                                          *
 *符号位          无                                          *
 *范围            位值标识                                    *
 *功能            每个按键对应一个bit                          *
 *                Bit 0:W键                                  *
 *                Bit 1:S键                                  *
 *                Bit 2:A键                                  *
 *                Bit 3:D键                                  *
 *                Bit 4:Shift键                              *
 *                Bit 5:Ctrl键                               *
 *                Bit 6:Q键                                  *
 *                Bit 7:E键                                  *
 *                Bit 8:R键                                  *
 *                Bit 9:F键                                  *
 *                Bit10:G键                                  *
 *                Bit11:Z键                                  *
 *                Bit12:X键                                  *
 *                Bit13:C键                                  *
 *                Bit14:V键                                  *
 *                Bit15:B键                                  *
 **************************************************************/
```

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
        <td align="center" rowspan=3>底盘控制</td>
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
        <td align="center" rowspan=3>机械臂控制</td>
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
