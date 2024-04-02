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
            }
        })
        .catch(error => {
            console.error('Error fetching data: ', error)
        })
}

setInterval(status_table_update, ALTAIR_MASTER_CLOCK_MS)