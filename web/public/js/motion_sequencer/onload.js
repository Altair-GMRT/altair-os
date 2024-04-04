function on_load_update() {
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
                        on_click_pose(poses[i])
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
}