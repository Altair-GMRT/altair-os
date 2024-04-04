function save_motion() {
    var file_name   = document.getElementById("filename_entry").value
    var seq_list    = []

    if(pose_cnt == 0) {
        return
    }

    for(let i = 0; i < pose_cnt; i++) {
        let seq = []

        seq.push(document.getElementById(`pose_name_${i}`).innerText)
        seq.push(parseInt(document.getElementById(`dly_${i}`).value, 10))
        seq.push(parseInt(document.getElementById(`dtn_${i}`).value, 10))

        seq_list.push(seq)
    }

    fetch(`${ALTAIR_URL}/api/save_motion`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            filename: file_name,
            val: seq_list
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


function cancel_save_motion() {
    document.getElementById("filename_overlay").classList.add("hidden");
    document.getElementById("filename").classList.add("hidden");
}