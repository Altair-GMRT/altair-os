function save_pose() {
    var file_name   = document.getElementById("filename_entry").value
    var pres_pos    = []

    for(let i = 0; i < ALTAIR_DXL_NUM; i++) {
        let val = parseInt(document.getElementById(`pos_${ALTAIR_DXL_ID[i]}`).innerHTML, 10)
        pres_pos.push(val)
        document.getElementById(`spos_${ALTAIR_DXL_ID[i]}`).innerHTML = val
    }

    fetch(`${ALTAIR_URL}/api/save_pose`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            filename: file_name,
            val: pres_pos
        })
    })
        .then(response => response.json())
        .then(data => {
            console.log(data.message);
        })
        .catch(error => {
            console.error('Error: ', error);
        });

    
    document.getElementById("filename_overlay").classList.add("hidden");
    document.getElementById("filename").classList.add("hidden");
}


function cancel_save_pose() {
    document.getElementById("filename_overlay").classList.add("hidden");
    document.getElementById("filename").classList.add("hidden");
}