function on_load_update() {
    var torque_en = []

    for(let i = 0; i < ALTAIR_DXL_NUM; i++) {
        torque_en.push(0)
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

    for(let i = 0; i < ALTAIR_DXL_NUM; i++) {
        let tpos_element = document.getElementById(`tpos_${ALTAIR_DXL_ID[i]}`)

        tpos_element.addEventListener('keydown', (event) => {
            if(event.key == 'ArrowUp') {
                tpos_element.value = parseInt(tpos_element.value, 10) + 1
            }
            else if(event.key == 'ArrowDown') {
                tpos_element.value = parseInt(tpos_element.value, 10) - 1
            }
        })
    }

    document.getElementById('filename_entry').addEventListener('keypress', (event) => {
        if(event.key == 'Enter') {
            event.preventDefault()
            save_pose()
        }
    })
}