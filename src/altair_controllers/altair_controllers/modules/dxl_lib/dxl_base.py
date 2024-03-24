import dynamixel_sdk as dxl
from .dxl_type import *


class DXLBase:

    def __init__(self, porthandler:dxl.PortHandler, packethandler:dxl.Protocol1PacketHandler | dxl.Protocol2PacketHandler, baudrate:int=1000000) -> None:
        self.porthandler    = porthandler
        self.packethandler  = packethandler
        self.registered_id  = set(())
        self.dxl_info       = dict(())



    def addDxl(self, joint_name:str, dxl_id:int) -> None:
        self.registered_id.add(dxl_id)
        self.dxl_info.update({
            dxl_id: DXLType(
                name    = joint_name,
                dxl_id  = dxl_id
            )
        })



    def convert1ByteToDxl(self, data:int) -> list:
        return [
            dxl.DXL_LOBYTE(dxl.DXL_LOWORD(data))
        ]



    def convert2ByteToDxl(self, data:int) -> list:
        return [
            dxl.DXL_LOBYTE(dxl.DXL_LOWORD(data)), 
            dxl.DXL_HIBYTE(dxl.DXL_LOWORD(data))
        ]



    def convert4ByteToDxl(self, data:int) -> list:
        return [
            dxl.DXL_LOBYTE(dxl.DXL_LOWORD(data)),
            dxl.DXL_HIBYTE(dxl.DXL_LOWORD(data)),
            dxl.DXL_LOBYTE(dxl.DXL_HIWORD(data)),
            dxl.DXL_HIBYTE(dxl.DXL_HIWORD(data))
        ]