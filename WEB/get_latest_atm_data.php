<?php
// Detalii de conectare la baza de date
$servername = "localhost";
$username = "";
$password = "";
$dbname = "";

// Se creează conexiunea la baza de date
$conn = new mysqli($servername, $username, $password, $dbname);

// Se verifică conexiunea
if ($conn->connect_error) {
    die("Connection failed: " . $conn->connect_error);
}

// Se execută interogarea pentru a extrage ultima înregistrare
$sql = "SELECT * FROM atm_data ORDER BY id DESC LIMIT 1";
$result = $conn->query($sql);

if ($result->num_rows > 0) {
    // Datele au fost găsite, sunt returnate în format JSON
    $row = $result->fetch_assoc();
    echo json_encode($row);
} else {
    echo json_encode(["error" => "No data found"]);
}

// Se închide conexiunea
$conn->close();
?>
