const path      = require('path')
const express   = require('express')
const config    = require('./src/js/config')
const rclga     = require('./src/js/rcl-gateaway')

const app       = express()
const port      = 3000
const rclnode   = new rclga()

app.set('view engine', 'ejs')
app.set('views', path.join(__dirname, './views'))
app.use(express.static(path.join(__dirname, './public')))
app.use(express.urlencoded({ extended: false }))
app.use(express.json())



app.get(`/${config.robot_config.id}`, (req, res) => {
    res.render('pages/index', {
        page_name: 'Welcome',
        robot_config: config.robot_config,
        joint_config: config.joint_config
    })
})

app.get(`/${config.robot_config.id}/status`, (req, res) => {
    res.render('pages/status', {
        page_name: 'Status',
        robot_config: config.robot_config,
        joint_config: config.joint_config
    })
})

app.get(`/${config.robot_config.id}/pose_studio`, (req, res) => {
    res.render('pages/pose_studio', {
        page_name: 'Pose Studio',
        robot_config: config.robot_config,
        joint_config: config.joint_config
    })
})

app.get(`/${config.robot_config.id}/motion_sequencer`, (req, res) => {
    res.render('pages/motion_sequencer', {
        page_name: 'Motion Sequencer',
        robot_config: config.robot_config,
        joint_config: config.joint_config
    })
})



const svr = app.listen(port, () => {
    console.log(`Altair App is running on ${config.robot_config.ip}:${port}`)
})



process.on('SIGINT', () => {
    console.log('\nClosing Altair App...')

    svr.close(() => {
        console.log('Altair app shutted down.')
    })
})