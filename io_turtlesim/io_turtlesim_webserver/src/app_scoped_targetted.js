var app = new Vue({
    el: '#app',
    data: {
        address: env.WS_URL,
        state: 'disconnected',
        ros: {},
        ros_components: {},
        // ros_component: {
        //     cmd_vel: null,
        //     listener: null,
        //     teleport: null
        // },
        id: 0,
        teleport: null,
        update_interval: null,
        turtles: {},
        vel_req: {
            name: 0,
            linear_velocity: 0.0,
            angular_velocity: 0.0
        },
        loop_move: null,
        teleport_req: {
            name: 0,
            x: 0.0,
            y: 0.0,
            theta: 0.0
        },
        goto_action_req: {
            name: 0,
            x: 0.0,
            y: 0.0
        },
        background: {
            blue: 255,
            green: 86,
            red: 69
        },
        pen: {
            blue: 255,
            green: 184,
            red: 179,
            width: 3
        },
        pen_off: false,
        bg_style: {
            position: 'relative',
            width: '545px',
            height: '545px',
            backgroundColor: 'rgb(0, 0, 0)',
            border: '22.5px solid white'
        },
        canvas_style: {
            position: 'absolute',
            top: '0',
            left: '0',
            zIndex: '0',
            width: '500px',
            height: '500px'
        },
        turtle_style: {
            position: 'absolute',
            marginLeft: '-22.5px',
            marginTop: '-22.5px',
            width: '45px',
            height: '45px'
        },
        stroke: {
            line_width: 0.0,
            stroke_style: 'rgb(0, 0, 0)'
        },
        canvas_prop: {
            px_ratio: 25,
            width: 500,
            height: 500
        }
    },
    methods: {
        connect() {
            var vm = this;
            vm.ros = new ROSLIB.Ros({
                url: vm.address
            });

            vm.ros.on('connection', function() {
                vm.state = 'success';
                vm.ros_init();
                // vm.get_pose();
               vm.update_interval = setInterval(vm.update_topic_list, 1000)
            });

            vm.ros.on('error', function(error) {
                vm.state = 'disconnected';
                console.log('Error connecting to websocket server: ', error);
            });

            vm.ros.on('close', function() {
                vm.state = 'disconnected';
                vm.reset();
                clearInterval(vm.update_interval)
                console.log('Connection to websocket server closed.');
            });
        },
        ros_init() {
            var vm = this;
            vm.teleport = new ROSLIB.Service({
                ros: vm.ros,
                name: '/teleport_turtle',
                serviceType: 'io_turtle_services/TeleportTurtle'
            })
        },
        ros_add_components(name) {
            var vm = this;
            vm.ros_components[name] = {
                cmd_vel: new ROSLIB.Topic({
                                ros: vm.ros,
                                name: '/'+name+'/cmd_vel',
                                messageType: 'geometry_msgs/Twist'
                            }),
                listener: new ROSLIB.Topic({
                                ros: vm.ros,
                                name: '/'+name+'/pose',
                                messageType: 'geometry_msgs/Pose2D'
                            }),
                // teleport: new ROSLIB.Service({
                //                 ros: vm.ros,
                //                 name: '/'+name+'/teleport_turtle',
                //                 serviceType: 'io_turtle_services/TeleportTurtle'
                //             })
            }
            Vue.set(vm.turtles, name, {
                image: './img/turtle'+(vm.id % 8) +'.png',
                pose: {
                    x: 0,
                    y: 0,
                    theta: 0
                },
                action: new ROSLIB.ActionClient({
                    ros: vm.ros,
                    serverName: '/'+ name +'/goto_action',
                    actionName: 'io_turtle_action/GoToAction'
                }),
                goal: {}
            });
            vm.get_pose(name)
            vm.id += 1

            console.log("New Turtle " + name + " added.");
        },
        update_topic_list() {
            var vm = this;
            vm.ros.getTopics((topics) => {
              // console.log("Getting topics...",);
              topics.topics.forEach(function(name) {
                const splitted = name.split('/')
                const pose_index = splitted.indexOf('pose')
                if( pose_index > 0 && splitted.length == 3 ){
                    var turtle_name = splitted[pose_index-1]
                    if (! vm.turtles.hasOwnProperty(turtle_name) && splitted[pose_index-1]!='sim') {
                        vm.ros_add_components(turtle_name)
                    }
                }
              }, this)

            })
        },
        move_once() {
            var vm = this;
            vm.publish_vel_cmd();
            setTimeout(() => { vm.move_stop() }, 999);
        },
        move_along() {
            var vm = this;
            vm.publish_vel_cmd();
        },
        move_stop() {
            var vm = this;
            var vel = new ROSLIB.Message({
                // id: parseInt(vm.vel_req.id),
                // linear_velocity: 0.0,
                // angular_velocity: 0.0
                linear: {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0
                },
                angular: {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0
                }
            });
            vm.ros_components[vm.vel_req.name].cmd_vel.publish(vel);
        },
        publish_vel_cmd() {
            var vm = this;
            var vel = new ROSLIB.Message({
                // id: parseInt(vm.vel_req.id),
                // linear_velocity: parseFloat(vm.vel_req.linear_velocity),
                // angular_velocity: vm.to_radians(parseFloat(vm.vel_req.angular_velocity))
                linear: {
                    x: parseFloat(vm.vel_req.linear_velocity),
                    y: 0.0,
                    z: 0.0
                },
                angular: {
                    x: 0.0,
                    y: 0.0,
                    z: vm.to_radians(parseFloat(vm.vel_req.angular_velocity))
                }
            });

            vm.ros_components[vm.vel_req.name].cmd_vel.publish(vel);

            console.log("velocity request for Turtle" + vm.vel_req.name +
                        " with command (" + vm.vel_req.linear_velocity +
                        ", " + vm.vel_req.angular_velocity +
                        ") was sent.");
        },
        set_color() {
            var vm = this;
            vm.bg_style.backgroundColor = 'rgb('+ vm.background.red + ', ' + vm.background.green + ', ' + vm.background.blue +')';
        },
        clear_board() {
            var canvas = document.getElementById("turtlesim_canvas");
            var ctx = canvas.getContext("2d");
            ctx.clearRect(0, 0, canvas.width, canvas.height);
        },
        to_radians(deg) {
            return deg * Math.PI / 180;
        },
        to_degrees(rad) {
            return rad * 180 / Math.PI;
        },
        get_pose(name) {
            var vm = this;
            vm.ros_components[name].listener.subscribe(function(message) {
                var prev_px_coord = vm.to_px_coord(vm.turtles[name].pose.x, vm.turtles[name].pose.y);
                vm.turtles[name].pose = message;
                var curr_px_coord = vm.to_px_coord(vm.turtles[name].pose.x, vm.turtles[name].pose.y);

                var turtle_elem = document.getElementById(name);
                var turtle_img_elem = document.getElementById('turtleImage_'+name);

                // console.log(prev_px_coord, curr_px_coord)
                if (turtle_elem == null || turtle_img_elem == null) {
                    console.log('null return', turtle_elem, turtle_img_elem)
                    return;
                }

                turtle_elem.style.paddingLeft = curr_px_coord.x + 'px';
                turtle_elem.style.paddingTop = curr_px_coord.y + 'px';

                // The transform function considers clockwise rotation as positive so we consider negative angle
                // The images are oriented vertically so we rotate them by 90 degrees to align with x axis
                turtle_img_elem.style.transform = 'rotate(' + (vm.to_degrees(-vm.turtles[name].pose.theta) + 90) + 'deg)';

                if (!vm.pen_off && vm.turtles[name].pose.linear_velocity !== 0.0) {
                    vm.draw_at_point(prev_px_coord, curr_px_coord);
                }
            });
        },
        to_px_coord(x, y) {
            var vm = this;
            var px = {
                x: vm.canvas_prop.width/2 + x * vm.canvas_prop.px_ratio,
                y: vm.canvas_prop.height/2 - y * vm.canvas_prop.px_ratio
            };
            return px;
        },
        draw_at_point(prev, curr) {
            var vm = this;
            var canvas = document.getElementById("turtlesim_canvas");
            var ctx = canvas.getContext("2d");
            ctx.beginPath();
            ctx.moveTo(prev.x, prev.y);
            ctx.lineTo(curr.x, curr.y);
            ctx.lineWidth = vm.stroke.line_width;
            ctx.strokeStyle = vm.stroke.stroke_style;
            ctx.stroke();
        },
        teleport_turtle() {
            vm = this;

            if (!vm.turtles[vm.teleport_req.name]) {
              console.log(vm.teleport_req.name + ' has not been registered.');
              return;
            }

            var request = new ROSLIB.ServiceRequest({
                id: parseInt(vm.teleport_req.name.split("turtle")[1]),
                x: parseFloat(vm.teleport_req.x),
                y: parseFloat(vm.teleport_req.y),
                theta: vm.to_radians(parseFloat(vm.teleport_req.theta))
            });

            console.log("Teleport request for " + vm.teleport_req.name +
                        " to (" + vm.teleport_req.x +
                        ", " + vm.teleport_req.y +
                        ", " + vm.teleport_req.theta +
                        ") was sent.");

            console.log(vm, vm.teleport)
            vm.teleport.callService(request, (result) => {
                if (result.data) {
                    console.log("Teleport request for " + vm.teleport_req.name +
                                " to (" + vm.teleport_req.x +
                                ", " + vm.teleport_req.y +
                                ", " + vm.teleport_req.theta +
                                ") was successful.");
                }
            });
        },
        goto_turtle_start() {
            vm = this;

            if (!vm.turtles[vm.goto_action_req.name]) {
              console.log(vm.goto_action_req.name + ' has not been registered.');
              return;
            }

            vm.turtles[vm.goto_action_req.name].goal = new ROSLIB.Goal({
                actionClient : vm.turtles[vm.goto_action_req.name].action,
                goalMessage : {
                  x : parseFloat(vm.goto_action_req.x),
                  y : parseFloat(vm.goto_action_req.y)
                }
            });
            vm.turtles[vm.goto_action_req.name].goal.on('feedback', function(feedback) {
                console.log('Feedback: ' + feedback.data);
            });
            vm.turtles[vm.goto_action_req.name].goal.on('result', function(result) {
                console.log('Result: ' + result.data);
            });
            vm.turtles[vm.goto_action_req.name].goal.send(5000);
            console.log("Goal request for " + vm.goto_action_req.name +
                        " to (" + vm.goto_action_req.x +
                        ", " + vm.goto_action_req.y +
                        ") was sent.");
        },
        goto_turtle_stop() {
            vm = this;

            if (!vm.turtles[vm.goto_action_req.name]) {
              console.log(vm.goto_action_req.name + ' has not been registered.');
              return;
            }

            vm.turtles[vm.goto_action_req.name].goal.cancel();
        },
        set_pen() {
            var vm = this;
            vm.stroke.line_width = parseInt(vm.pen.width);
            vm.stroke.stroke_style = 'rgb(' + parseInt(vm.pen.red) + ', ' + 
                parseInt(vm.pen.green) + ', ' + parseInt(vm.pen.blue) + ')';
        },
        pen_on_off() {
            var vm = this;
            vm.pen_off = !vm.pen_off;
            vm.set_pen();
        },
        reset() {
            var vm = this;
            vm.turtles = {};
        }
    },
    created() {
        var vm = this;
        vm.connect();
        vm.clear_board();
        vm.set_color();
        vm.set_pen();
    }
});
