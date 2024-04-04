var pose_cnt    = 0
var pose_list   = []


function on_create_pose(idx, fname) {
    var parent  = document.getElementById('sequencer_list')
    var tr_item = document.createElement('tr')
    var th_no   = document.createElement('th')
    var td_pose = document.createElement('td')
    var td_dly  = document.createElement('td')
    var in_dly  = document.createElement('input')
    var td_dtn  = document.createElement('td')
    var in_dtn  = document.createElement('input')
    var td_del  = document.createElement('td')
    var btn_del = document.createElement('button')

    tr_item.className = "border-b bg-gray-800 border-gray-700"
    tr_item.id = `pose_list_${idx}`

    th_no.setAttribute("scope", "row")
    th_no.className = "px-6 py-4 font-medium whitespace-nowrap text-white"
    th_no.innerText = `${idx + 1}`
    
    td_pose.className   = "px-6 py-4"
    td_pose.id          = `pose_name_${idx}`
    td_pose.innerText   = fname

    td_dly.className    = "px-6 py-4"
    in_dly.className    = "border text-sm rounded-lg block w-24 py-1 text-center bg-gray-700 border-gray-600 placeholder-gray-400 text-white focus:ring-blue-500 focus:border-blue-500"
    in_dly.id           = `dly_${idx}`
    in_dly.placeholder  = 0

    td_dtn.className    = "px-6 py-4"
    in_dtn.className    = "border text-sm rounded-lg block w-24 py-1 text-center bg-gray-700 border-gray-600 placeholder-gray-400 text-white focus:ring-blue-500 focus:border-blue-500"
    in_dtn.id           = `dtn_${idx}`
    in_dtn.placeholder  = 0

    td_del.className    = "px-6 py-4"
    btn_del.className   = "px-4 py-1 text-white rounded-xl border border-gray-700 hover:bg-red-600"
    btn_del.innerText   = 'X'
    
    const this_pose_idx = idx
    btn_del.onclick = function() {
        on_delete_pose(this_pose_idx)
    }

    td_dly.appendChild(in_dly)
    td_dtn.appendChild(in_dtn)

    td_del.appendChild(btn_del)
    tr_item.appendChild(th_no)

    tr_item.appendChild(td_pose)
    tr_item.appendChild(td_dly)
    tr_item.appendChild(td_dtn)
    tr_item.appendChild(td_del)

    parent.appendChild(tr_item)
}


function on_delete_pose(idx) {
    pose_cnt -= 1
    pose_list.splice(idx, 1)

    var parent = document.getElementById('sequencer_list')

    while(parent.firstChild) {
        parent.removeChild(parent.firstChild)
    }

    if(pose_cnt == 0) {
        parent.innerHTML += `
        <tr class="border-b bg-gray-800 border-gray-700" id="table_init">
            <th scope="row" class="px-6 py-4 font-medium whitespace-nowrap text-white">
                -
            </th>
            <td class="px-6 py-4">
                No Pose Selected
            </td>
            <td class="px-6 py-4">
                -
            </td>
            <td class="px-6 py-4">
                -
            </td>
            <td class="px-6 py-4">
                -
            </td>
        </tr>
        `
        return
    }

    for(let i = 0; i < pose_cnt; i++) {
        on_create_pose(i, pose_list[i])
    }
}


function on_click_pose(fname) {
    on_create_pose(pose_cnt, fname)

    pose_list.push(fname)
    pose_cnt += 1

    if(pose_cnt > 0) {
        document.getElementById('table_init').classList.add('hidden')
    }
}


function reset_motion() {
    let user_resp = confirm('Proceed to reset the motion?')
    
    if(user_resp) {
        var parent = document.getElementById('sequencer_list')

        while(parent.firstChild) {
            parent.removeChild(parent.firstChild)
        }

        parent.innerHTML += `
        <tr class="border-b bg-gray-800 border-gray-700" id="table_init">
            <th scope="row" class="px-6 py-4 font-medium whitespace-nowrap text-white">
                -
            </th>
            <td class="px-6 py-4">
                No Pose Selected
            </td>
            <td class="px-6 py-4">
                -
            </td>
            <td class="px-6 py-4">
                -
            </td>
            <td class="px-6 py-4">
                -
            </td>
        </tr>
        `

        pose_cnt    = 0
        pose_list   = []
    }
}


function on_click_motion(fname) {
    fetch(`${ALTAIR_URL}/api/get_motion_value`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            filename: fname
        })
    })
        .then(response => response.json())
        .then(data => {
            console.log(data)
            pose_cnt    = 0
            pose_list   = []

            var parent = document.getElementById('sequencer_list')

            while(parent.firstChild) {
                parent.removeChild(parent.firstChild)
            }

            if(data.val.length != 0) {
                for(let i = 0; i < data.val.length; i++) {
                    var tr_item = document.createElement('tr')
                    var th_no   = document.createElement('th')
                    var td_pose = document.createElement('td')
                    var td_dly  = document.createElement('td')
                    var in_dly  = document.createElement('input')
                    var td_dtn  = document.createElement('td')
                    var in_dtn  = document.createElement('input')
                    var td_del  = document.createElement('td')
                    var btn_del = document.createElement('button')

                    tr_item.className = "border-b bg-gray-800 border-gray-700"
                    tr_item.id = `pose_list_${i}`

                    th_no.setAttribute("scope", "row")
                    th_no.className = "px-6 py-4 font-medium whitespace-nowrap text-white"
                    th_no.innerText = `${i + 1}`
                    
                    td_pose.className   = "px-6 py-4"
                    td_pose.id          = `pose_name_${i}`
                    td_pose.innerText   = data.val[i][0]

                    td_dly.className    = "px-6 py-4"
                    in_dly.className    = "border text-sm rounded-lg block w-24 py-1 text-center bg-gray-700 border-gray-600 placeholder-gray-400 text-white focus:ring-blue-500 focus:border-blue-500"
                    in_dly.id           = `dly_${i}`
                    in_dly.placeholder  = 0
                    in_dly.value        = data.val[i][1]

                    td_dtn.className    = "px-6 py-4"
                    in_dtn.className    = "border text-sm rounded-lg block w-24 py-1 text-center bg-gray-700 border-gray-600 placeholder-gray-400 text-white focus:ring-blue-500 focus:border-blue-500"
                    in_dtn.id           = `dtn_${i}`
                    in_dtn.placeholder  = 0
                    in_dtn.value        = data.val[i][2]

                    td_del.className    = "px-6 py-4"
                    btn_del.className   = "px-4 py-1 text-white rounded-xl border border-gray-700 hover:bg-red-600"
                    btn_del.innerText   = 'X'
                    
                    const this_pose_idx = i
                    btn_del.onclick = function() {
                        on_delete_pose(this_pose_idx)
                    }

                    td_dly.appendChild(in_dly)
                    td_dtn.appendChild(in_dtn)

                    td_del.appendChild(btn_del)
                    tr_item.appendChild(th_no)

                    tr_item.appendChild(td_pose)
                    tr_item.appendChild(td_dly)
                    tr_item.appendChild(td_dtn)
                    tr_item.appendChild(td_del)

                    parent.appendChild(tr_item)

                    pose_list.push(data.val[i][0])
                    pose_cnt += 1
                }
            }

            else {
                parent.innerHTML += `
                    <tr class="border-b bg-gray-800 border-gray-700" id="table_init">
                        <th scope="row" class="px-6 py-4 font-medium whitespace-nowrap text-white">
                            -
                        </th>
                        <td class="px-6 py-4">
                            No Pose Selected
                        </td>
                        <td class="px-6 py-4">
                            -
                        </td>
                        <td class="px-6 py-4">
                            -
                        </td>
                        <td class="px-6 py-4">
                            -
                        </td>
                    </tr>
                `
            }
        })
        .catch(error => {
            console.error('Error: ', error);
        });

    document.getElementById('motion_select_overlay').classList.add('hidden')
    document.getElementById('motion_select').classList.add('hidden')
}


function open_saved_motions() {
    fetch(`${ALTAIR_URL}/api/get_saved_motions`)
        .then(response => {
            if(!response.ok) {
                throw new Error(`Bad response from ${ALTAIR_URL}/api/get_saved_motions`)
            }
            return response.json()
        })
        .then(data => {
            var motions = data.motions
            var parent  = document.getElementById('motion_item')

            while(parent.firstChild) {
                parent.removeChild(parent.firstChild)
            }

            if(motions.length != 0) {
                for(let i = 0; i < motions.length; i++) {
                    let item    = document.createElement('tr')
                    let no      = document.createElement('th')
                    let file    = document.createElement('td')

                    item.className  = 'bg-gray-800 border-b border-gray-700 hover:bg-gray-600'
                    no.className    = 'px-3 py-3 font-medium text-white whitespace-nowrap'
                    file.className  = 'px-20 py-3 font-medium text-white'

                    item.addEventListener('click', () => {
                        on_click_motion(motions[i])
                    })

                    no.innerHTML    = `${i + 1}`
                    file.innerHTML  = `${motions[i]}`

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
                            No motion file exists
                        </td>
                    </tr>
                `
            }
        })
        .catch(error => {
            console.error('Error fetching data: ', error)
        })

    document.getElementById('motion_select_overlay').classList.remove('hidden')
    document.getElementById('motion_select').classList.remove('hidden')
}


function show_save_prompt() {
    document.getElementById('filename_overlay').classList.remove('hidden')
    document.getElementById('filename').classList.remove('hidden')
}


function play_motion() {

}