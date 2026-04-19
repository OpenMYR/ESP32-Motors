var httpRequest = new XMLHttpRequest();

document.getElementById("connect").addEventListener("click", function(){
	var parameters={
		code : "C",
		data : [
			document.getElementById("ssid").value,
			document.getElementById("pass").value
		]
	};
	if (parameters.data[0] !== ""){
		send(parameters);
	} else {
		alert("\"Network Name\" cannot be left blank");
	}
});

document.getElementById("disconnect").addEventListener("click", function(){
	var parameters={
		code : "D",
		data : [
			"",
			""
		]
	};
	send(parameters);
});

var changeStatus = document.getElementById("changeStatus");

function updateChangeStatus(message, success) {
	changeStatus.textContent = message || "";
	changeStatus.classList.remove("success", "error");
	if (success === true) {
		changeStatus.classList.add("success");
	} else if (success === false) {
		changeStatus.classList.add("error");
	}
}

document.getElementById("change").addEventListener("click", function(){
	var oldPass = document.getElementById("oldOTAPass").value;
	var newPass = document.getElementById("newOTAPass1").value;
	var confirmPass = document.getElementById("newOTAPass2").value;

	if (newPass !== confirmPass) {
		alert("Confrimation password does not match");
		return;
	}
	if (!oldPass || !newPass) {
		alert("Old and new OTA password are required");
		return;
	}

	var parameters={
		code : "O",
		data: [
			oldPass,
			newPass
		]
	};
	updateChangeStatus("Updating OTA password...", null);
	send(parameters, function(request){
		if (request.status === 202) {
			updateChangeStatus("OTA password updated.", true);
		} else {
			var message = request.responseText || request.statusText || request.status;
			updateChangeStatus("Change failed: " + message, false);
		}
	});
});

document.getElementById("otaForm").addEventListener("submit", async function(event){
	event.preventDefault();
	var user = document.getElementById("otaUser").value.trim();
	var pass = document.getElementById("otaPass").value;
	var fileInput = document.getElementById("firmware");
	var status = document.getElementById("otaStatus");
	var uploadButton = document.getElementById("upload");

	if (!user || !pass || !fileInput.files.length) {
		status.textContent = "Username, password, and firmware file are required.";
		return;
	}

	var file = fileInput.files[0];
	var token = btoa(user + ":" + pass);
	uploadButton.disabled = true;
	status.textContent = "Uploading firmware...";

	try{
		var response = await fetch("/ota", {
			method: "POST",
			headers: {
				"Authorization": "Basic " + token,
				"Content-Type": "application/octet-stream"
			},
			body: file
		});
		var text = await response.text();
		if (response.ok){
			status.textContent = text || "Upload complete. Device rebooting...";
		} else {
			status.textContent = "Upload failed: " + (text || response.status);
		}
	} catch (error) {
		status.textContent = "Upload error: " + error;
	}

	uploadButton.disabled = false;
});

function send (args, callback){
	let out = {
		commands : [args]
	};
	httpRequest.open("POST", "/", true);
	httpRequest.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
	httpRequest.onreadystatechange = function(){
		if (httpRequest.readyState !== XMLHttpRequest.DONE) return;
		if (callback) callback(httpRequest);
	};
	httpRequest.send(JSON.stringify(out));
}