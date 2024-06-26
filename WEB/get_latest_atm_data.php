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

// Execută interogarea pentru a extrage ultima înregistrare
$sql = "SELECT * FROM atm_data ORDER BY id DESC LIMIT 1";
$result = $conn->query($sql);

if ($result->num_rows > 0) {
    // Datele au fost găsite, le returnăm în format JSON
    $row = $result->fetch_assoc();
    echo json_encode($row);
} else {
    echo json_encode(["error" => "No data found"]);
}

// Închide conexiunea
$conn->close();
?>
