<?php
// Detalii de conectare la baza de date
$servername = "localhost";
$username = "u198184306_stm32_data";
$password = "Aprozar-12"; 
$dbname = "u198184306_stm32_data";

// Creează conexiunea la baza de date
$conn = new mysqli($servername, $username, $password, $dbname);

// Verifică conexiunea
if ($conn->connect_error) {
    die("Connection failed: " . $conn->connect_error);
}

// Verifică dacă datele sunt trimise prin metoda POST
if ($_SERVER['REQUEST_METHOD'] === 'POST') {
    $temperature = $_POST['temperature'] ?? null;
    $humidity = $_POST['humidity'] ?? null;
    $pressure = $_POST['pressure'] ?? null;
    $UVIndex = $_POST['UVIndex'] ?? null;
    $eCO2 = $_POST['eCO2'] ?? null;
    $TVOC = $_POST['TVOC'] ?? null;

    // Verifică dacă toate variabilele necesare sunt setate
    if (isset($temperature, $humidity, $pressure, $UVIndex, $eCO2, $TVOC)) {
        // Pregătește și execută interogarea SQL pentru inserarea datelor
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

// Închide conexiunea
$conn->close();
?>
