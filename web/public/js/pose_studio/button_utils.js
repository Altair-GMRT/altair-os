function on_click_pose(fname) {
    fetch(`${ALTAIR_URL}/api/get_pose_value`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            filename: fname
        })
    })
        .then(response => response.json())
        .then(data => {
            for(let i = 0; i < ALTAIR_DXL_NUM; i++) {
                document.getElementById(`spos_${ALTAIR_DXL_ID[i]}`).innerHTML = data.val[i]
            }
        })
        .catch(error => {
            console.error('Error: ', error);
        });

    document.getElementById('pose_select_overlay').classList.add('hidden')
    document.getElementById('pose_select').classList.add('hidden')
}


function show_saved_poses() {
    fetch(`${ALTAIR_URL}/api/get_saved_poses`)
        .then(response => {
            if(!response.ok) {
                throw new Error(`Bad response from ${ALTAIR_URL}/api/get_saved_poses`)
            }
            return response.json()
        })
        .then(data => {
            var poses   = data.poses
            var parent  = document.getElementById('pose_item')

            while(parent.firstChild) {
                parent.removeChild(parent.firstChild)
            }

            if(poses.length != 0) {
                for(let i = 0; i < poses.length; i++) {
                    let item    = document.createElement('tr')
                    let no      = document.createElement('th')
                    let file    = document.createElement('td')

                    item.className  = 'bg-gray-800 border-b border-gray-700 hover:bg-gray-600'
                    no.className    = 'px-3 py-3 font-medium text-white whitespace-nowrap'
                    file.className  = 'px-20 py-3 font-medium text-white'

                    item.addEventListener('click', () => {
                        on_click_pose(poses[i])
                    })

                    no.innerHTML    = `${i + 1}`
                    file.innerHTML  = `${poses[i]}`

                    item.appendChild(no)
                    item.appendChild(file)
                    parent.appendChild(item)
                }
            }

            else {
                parent.innerHTML += `
                    <tr class="bg-gray-800 border-b border-gray-700 hover:bg-gray-600">
                        <th scope="row" class="px-3 py-3 font-medium text-white whitespace-nowrap">
                            -
                        </th>
                        <td class="px-20 py-3 text-white">
                            No pose file exists
                        </td>
                    </tr>
                `
            }
        })
        .catch(error => {
            console.error('Error fetching data: ', error)
        })

    document.getElementById('pose_select_overlay').classList.remove('hidden')
    document.getElementById('pose_select').classList.remove('hidden')
}


function show_save_prompt() {
    document.getElementById('filename_overlay').classList.remove('hidden')
    document.getElementById('filename').classList.remove('hidden')
}


function play_pose() {
    for(let i = 0; i < ALTAIR_DXL_NUM; i++) {
        document.getElementById(`tpos_${ALTAIR_DXL_ID[i]}`).value = document.getElementById(`spos_${ALTAIR_DXL_ID[i]}`).innerText
    }
}


function enable_all_torque() {
    for(let i = 0; i < ALTAIR_DXL_NUM; i++) {
        document.getElementById(`torque_en_${ALTAIR_DXL_ID[i]}`).checked = true    
    }
}


function disable_all_torque() {
    for(let i = 0; i < ALTAIR_DXL_NUM; i++) {
        document.getElementById(`torque_en_${ALTAIR_DXL_ID[i]}`).checked = false
    }
}


function apply_torque() {
    var torque_en           = []
    var check_num           = 0
    let play_pose_button    = document.getElementById("play_pose_button")

    for(let i = 0; i < ALTAIR_DXL_NUM; i++) {
        let checked = document.getElementById(`torque_en_${ALTAIR_DXL_ID[i]}`).checked
        check_num   = (checked) ? check_num + 1 : check_num
        torque_en.push((checked) ? 1 : 0)
    }

    if(check_num == ALTAIR_DXL_NUM) {
        play_pose_button.disabled = false
        play_pose_button.classList.replace("text-gray-950", "text-white")
    }
    else {
        play_pose_button.disabled = true
        play_pose_button.classList.replace("text-white", "text-gray-950")
    }

    fetch(`${ALTAIR_URL}/api/set_torque`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({goal_torque: torque_en})
    })
        .then(response => response.json())
        .then(data => {
            console.log(data.message);

            for(let i = 0; i < ALTAIR_DXL_NUM; i++) {
                var tpos_entry = document.getElementById(`tpos_${ALTAIR_DXL_ID[i]}`)

                if(torque_en[i] == 1) {
                    tpos_entry.value    = document.getElementById(`pos_${ALTAIR_DXL_ID[i]}`).innerText
                    tpos_entry.disabled = false
                    tpos_entry.classList.replace('text-gray-600', 'text-white')
                }

                else {
                    tpos_entry.value    = 0
                    tpos_entry.disabled = true
                    tpos_entry.classList.replace('text-white', 'text-gray-600')
                }
            }
        })
        .catch(error => {
            console.error('Error:', error);
        });
}