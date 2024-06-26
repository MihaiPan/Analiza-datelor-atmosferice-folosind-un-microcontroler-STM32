<?php
// Detalii de conectare la baza de date
$servername = "localhost";
$username = "";
$password = ""; 
$dbname = "";

// Creează conexiunea la baza de date
$conn = new mysqli($servername, $username, $password, $dbname);

// Verifică conexiunea
if ($conn->connect_error) {
    die("Connection failed: " . $conn->connect_error);
}

// Execută interogarea pentru a extrage ultima înregistrare
$sql = "SELECT * FROM location ORDER BY id DESC LIMIT 1";
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
