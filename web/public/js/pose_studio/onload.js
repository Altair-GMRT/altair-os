function onload_update() {
    fetch(`${ALTAIR_URL}/api/get_status`)
        .then(response => {
            if(!response.ok) {
                throw new Error(`Bad response from ${ALTAIR_URL}/api/get_status`)
            }
            return response.json()
        })
        .then(data => {
            var check_num = 0

            for(let i = 0; i < ALTAIR_DXL_NUM; i++) {
                var tpos_entry = document.getElementById(`tpos_${ALTAIR_DXL_ID[i]}`)

                if(data.present_torque[i] == 1) {
                    tpos_entry.value    = document.getElementById(`pos_${ALTAIR_DXL_ID[i]}`).innerText
                    tpos_entry.disabled = false
                    tpos_entry.classList.replace('text-gray-600', 'text-white')
                }

                else {
                    tpos_entry.value    = 0
                    tpos_entry.disabled = true
                    tpos_entry.classList.replace('text-white', 'text-gray-600')
                }

                document.getElementById(`torque_en_${ALTAIR_DXL_ID[i]}`).checked = data.present_torque[i]
                check_num = (data.present_torque[i] == 1) ? check_num + 1 : check_num
            }

            if(check_num == ALTAIR_DXL_NUM) {
                let play_pose_button = document.getElementById("play_pose_button")
                play_pose_button.disabled = false
                play_pose_button.classList.replace("text-gray-950", "text-white")
            }
        })
        .catch(error => {
            console.error('Error fetching data: ', error)
        })
}