function show_saved_pose() {
    fetch(`${ALTAIR_URL}/api/get_saved_pose`)
            .then(response => {
                if(!response.ok) {
                    throw new Error(`Bad response from ${ALTAIR_URL}/api/get_saved_pose`)
                }
                return response.json()
            })
            .then(data => {
                var poses   = data.poses
                var parent  = document.getElementById('pose_item')

                if(poses.length != 0) {
                    
                    while(parent.firstChild) {
                        parent.removeChild(parent.firstChild)
                    }

                    for(let i = 0; i < poses.length; i++) {
                        let item    = document.createElement('tr')
                        let no      = document.createElement('th')
                        let file    = document.createElement('td')

                        item.className  = 'bg-gray-800 border-b border-gray-700 hover:bg-gray-600'
                        no.className    = 'px-3 py-3 font-medium text-white whitespace-nowrap'
                        file.className  = 'px-20 py-3 font-medium text-white'

                        item.addEventListener('click', () => {
                            document.getElementById("pose_select_overlay").classList.add("hidden");
                            document.getElementById("pose_select").classList.add("hidden");
                        })

                        no.innerHTML    = `${i + 1}`
                        file.innerHTML  = `${poses[i]}`

                        item.appendChild(no)
                        item.appendChild(file)
                        parent.appendChild(item)
                    }
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
        play_pose_button.classList.remove("text-gray-950")
        play_pose_button.classList.add("text-white")
    }
    else {
        play_pose_button.disabled = true
        play_pose_button.classList.add("text-gray-950")
        play_pose_button.classList.remove("text-white")
    }

    fetch(`${ALTAIR_URL}/api/set_torque`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({goal_torque: torque_en})
    })
        .then(response => response.json())
        .then(data => {
            console.log(data.message);
        })
        .catch(error => {
            console.error('Error:', error);
        });
}