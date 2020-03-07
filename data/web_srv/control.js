var motorCanvas = [];
var angleCanvas = [];
var hornCanvas = [];
var motorParam = [];
var motors = [];

var vm = new VirtualMotor({
	traverseSpeed : 500,
	transmitInterval : 50,
	motorPositions : [90, 90, 90, 90]
});


for (let i = 1; i <= 15; i++) {
	motorCanvas.push(document.getElementById("motorCanvas" + i));
	angleCanvas.push(document.getElementById("angleCanvas" + i));
	hornCanvas.push(document.getElementById("hornCanvas" + i));
	motorParam.push({
		name: "Motor " + i,
		motorid: i,
		scale: 4,
		x: 150,
		y: 150,
		maxAngle: 180,
		minAngle: 0,
		angle: 90,
		hornLength: 100 
	});

	let sm = new ServoMotor({
		motorCanvas : motorCanvas[i - 1],
		angleCanvas : angleCanvas[i - 1],
		hornCanvas : hornCanvas[i - 1],
	}, motorParam[i - 1], vm);

	motors.push(sm);
}