<?php
$servername = "localhost";
$username = "";
$password = ""; 
$dbname = "";

$conn = new mysqli($servername, $username, $password, $dbname);

if ($conn->connect_error) {
    die("Connection failed: " . $conn->connect_error);
}

if ($_SERVER['REQUEST_METHOD'] === 'POST') {
    $temperature = $_POST['temperature'] ?? null;
    $humidity = $_POST['humidity'] ?? null;
    $pressure = $_POST['pressure'] ?? null;
    $UVIndex = $_POST['UVIndex'] ?? null;
    $eCO2 = $_POST['eCO2'] ?? null;
    $TVOC = $_POST['TVOC'] ?? null;

    if (isset($temperature, $humidity, $pressure, $UVIndex, $eCO2, $TVOC)) {
        $sql = "INSERT INTO atm_data (temperature, humidity, pressure, UVIndex, eCO2, TVOC) 
                VALUES ('$temperature', '$humidity', '$pressure', '$UVIndex', '$eCO2', '$TVOC')";

        if ($conn->query($sql) === TRUE) {
            echo "New record created successfully";
        } else {
            echo "Error: " . $sql . "<br>" . $conn->error;
        }
    } else {
        echo "Incomplete data received";
    }
} else {
    echo "No data received";
}

$conn->close();
?>
