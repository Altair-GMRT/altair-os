class DXLType:

    def __init__(self, name:str, dxl_id:int) -> None:
        self.name           = name
        self.id             = dxl_id
        self.torque_en      = 0
        self.goal_pos       = 0
        self.goal_vel       = 0
        self.present_pos    = 0
        self.present_vel    = 0
        self.present_load   = 0.0