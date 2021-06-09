# mgmt_lb
mgmt_lb for CableWalker Auto Navigation System

2021/06/09 Nikita Rodichenko: "mgmt_lb и CDrone устарели"

Wiki
In CMGMT.py:

if (abs(msg.y) < 0.2) and self.do_auto_land:
    # decr = -0.1
    print("DESCENDING")
    self.offset_alt = self.offset_alt - 0.05
    if (abs(msg.z) < 0.35) and (abs(msg.y) < 0.1) and (not self.full_land):
        self.full_land = True
        msg = self.connection.master.mav.command_long_encode(
            self.srcSystem,  # target_system
            1,  # uint8_t target_component
            mavlink.MAV_CMD_NAV_LAND,
            0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0
        )
        msg.pack(self.connection.master.mav)
        self.send_mavlink_message(msg)

(abs(msg.y) < 0.2)
- это условие работы автоснижения - 0.2 максимальное оцениваемое отклонение поперек провода. Можно увеличивать - тогда будет меньше "тупить", но снижение будет менее точным

self.offset_alt = self.offset_alt - 0.05
0.05 - коэффициент скорости снижения, можно увеличивать

(abs(msg.z) < 0.35)
0.35 - минимальное расстояние над проводом, когда включится автопосадка. Чтобы быстрее уходить в посадку, надо увеличивать. Если включать автопосалку на большей высоте, будет больше влиять ветер и неустойчивость платформы.

(abs(msg.y) < 0.1)
0.1 - максимальное оцениваемое отклонение поперек провода, чтобы активировался режим автостыковки. Если его увеличить, то автостыковка может включиться при большем отклонении от провода.

После включения автостыковки дрон не корректирует позицию отеосительно провода, садится вертикально вниз по ГНСС
