function status_table_update() {
    fetch(`${ALTAIR_URL}/api/get_status`)
        .then(response => {
            if(!response.ok) {
                throw new Error(`Bad response from ${ALTAIR_URL}/api/get_status`)
            }
            return response.json()
        })
        .then(data => {
            for(let i = 0; i < ALTAIR_DXL_NUM; i++) {
                document.getElementById(`pos_${ALTAIR_DXL_ID[i]}`).innerHTML = data.present_position[i]
                document.getElementById(`vel_${ALTAIR_DXL_ID[i]}`).innerHTML = data.present_velocity[i]
                document.getElementById(`temp_${ALTAIR_DXL_ID[i]}`).innerHTML = data.joint_sensor.temperature[i].toFixed(2)
                document.getElementById(`volt_${ALTAIR_DXL_ID[i]}`).innerHTML = data.joint_sensor.voltage[i].toFixed(2)
                document.getElementById(`load_${ALTAIR_DXL_ID[i]}`).innerHTML = data.joint_sensor.load[i].toFixed(2)
            }
        })
        .catch(error => {
            console.error('Error fetching data: ', error)
        })
}

setInterval(status_table_update, ALTAIR_MASTER_CLOCK_MS)