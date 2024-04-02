function change_view() {
    document.getElementById('table_a1').classList.toggle('hidden')
    document.getElementById('table_a2').classList.toggle('hidden')
    document.getElementById('table_b1').classList.toggle('hidden')
    document.getElementById('table_b2').classList.toggle('hidden')
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
    var torque_en = []

    for(let i = 0; i < ALTAIR_DXL_NUM; i++) {
        torque_en.push(
            (document.getElementById(`torque_en_${ALTAIR_DXL_ID[i]}`).checked) ? 1 : 0
        )
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