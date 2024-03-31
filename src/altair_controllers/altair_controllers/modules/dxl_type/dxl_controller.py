import dynamixel_sdk as dxl
from dxl_table import *



class DXLController:

    def __init__(self, porthandler:dxl.PortHandler, packethandler:dxl.Protocol1PacketHandler | dxl.Protocol2PacketHandler) -> None:
        self.porthandler    = porthandler
        self.packethandler  = packethandler



    def write(self, address:int, size:int, dxl_id:int, param:int) -> int:
        if size == 1:
            dxl_comm_res, dxl_err = self.packethandler.write1ByteTxRx(
                self.porthandler,
                dxl_id,
                address,
                param
            )

        elif size == 2:
            dxl_comm_res, dxl_err = self.packethandler.write2ByteTxRx(
                self.porthandler,
                dxl_id,
                address,
                param
            )

        elif size == 4:
            dxl_comm_res, dxl_err = self.packethandler.write4ByteTxRx(
                self.porthandler,
                dxl_id,
                address,
                param
            )

        else:
            return DXL_INVALID_SIZE
        
        if dxl_comm_res != dxl.COMM_SUCCESS:
            return DXL_COMM_ERROR

        elif dxl_err != 0:
            return DXL_PACKET_ERROR
        
        return DXL_OK


    
    def read(self, address:int, size:int, dxl_id:int) -> int:
        if size == 1:
            data, dxl_comm_res, dxl_err = self.packethandler.read1ByteTxRx(
                self.porthandler,
                dxl_id,
                address
            )

        elif size == 2:
            data, dxl_comm_res, dxl_err = self.packethandler.read2ByteTxRx(
                self.porthandler,
                dxl_id,
                address
            )

        elif size == 4:
            data, dxl_comm_res, dxl_err = self.packethandler.read4ByteTxRx(
                self.porthandler,
                dxl_id,
                address
            )

        else:
            return DXL_INVALID_SIZE
        
        if dxl_comm_res != dxl.COMM_SUCCESS:
            return DXL_COMM_ERROR

        elif dxl_err != 0:
            return DXL_PACKET_ERROR
        
        return data



    def groupSyncWrite(self, address:int, size:int, dxl_id:list, params:list) -> int:
        param_num = len(dxl_id)

        group_sync_write = dxl.GroupSyncWrite(
            self.porthandler,
            self.packethandler,
            address,
            size
        )

        for i in range(param_num):
            group_sync_write.addParam(dxl_id[i], params[i])

        dxl_comm_res = group_sync_write.txPacket()
        
        if dxl_comm_res != dxl.COMM_SUCCESS:
            return DXL_COMM_ERROR

        group_sync_write.clearParam()
        return DXL_OK



    def groupSyncRead(self, address:int, size:int, dxl_id:list, error_bypass:bool=False) -> list | int:
        data_res = []

        group_sync_read = dxl.GroupSyncRead(
            self.porthandler,
            self.packethandler,
            address,
            size
        )

        for id in dxl_id:
            group_sync_read.addParam(id)

        dxl_comm_res = group_sync_read.txRxPacket()

        if dxl_comm_res != dxl.COMM_SUCCESS:
            return DXL_COMM_ERROR
        
        for id in dxl_id:
            dxl_res = group_sync_read.isAvailable(id, address, size)
            
            if dxl_res != True:

                if not error_bypass:
                    return DXL_DATA_UNAVAILABLE
                
                else:
                    data_res.append(None)
            
            else:
                data_res.append(group_sync_read.getData(id, address, size))
        
        return data_res



    def groupBulkWrite(self, address:list, size:list, dxl_id:list, params:list) -> int:
        param_num = len(dxl_id)

        group_bulk_write = dxl.GroupBulkWrite(
            self.porthandler,
            self.packethandler
        )

        for i in range(param_num):
            group_bulk_write.addParam(
                dxl_id[i],
                address[i],
                size[i],
                params[i]
            )

        dxl_comm_res = group_bulk_write.txPacket()

        if dxl_comm_res != dxl.COMM_SUCCESS:
            return DXL_COMM_ERROR
        
        group_bulk_write.clearParam()



    def groupBulkRead(self, address:list, size:list, dxl_id:list, error_bypass:bool=False) -> list | int:
        param_num   = len(dxl_id)
        data_res    = []

        group_bulk_read = dxl.GroupBulkRead(
            self.porthandler,
            self.packethandler
        )

        for i in range(param_num):
            group_bulk_read.addParam(
                dxl_id[i],
                address[i],
                size[i]
            )

        dxl_comm_res = group_bulk_read.txRxPacket()

        if dxl_comm_res != dxl.COMM_SUCCESS:
            return DXL_COMM_ERROR
        
        for i in range(param_num):
            dxl_res = group_bulk_read.isAvailable(dxl_id[i], address[i], size[i])
            
            if dxl_res != True:

                if not error_bypass:
                    return DXL_DATA_UNAVAILABLE
                
                else:
                    data_res.append(None)

            else:
                data_res.append(group_bulk_read.getData(dxl_id[i], address[i], size[i]))

        return data_res
            


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