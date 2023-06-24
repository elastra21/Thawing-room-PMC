const express = require('express');
const app = express();

// Endpoint to download the .ota file
app.get('/download', (req, res) => {
  const filePath = './data/firmware.ota'; // Update with the actual path to the .ota file

  // Set appropriate headers for the file download
  res.setHeader('Content-disposition', 'attachment; filename=firmware.ota');
  res.setHeader('Content-type', 'application/octet-stream');

  // Send the file as the response
  res.download(filePath, (err) => {
    if (err) {
      console.error('Error downloading .ota file:', err);
      res.status(500).send('Error downloading .ota file');
    }
  });
});

// Endpoint to check the current version
app.get('/version', (req, res) => {
  const currentVersion = '1.0.0'; // Update with the actual current version

  res.json({ version: currentVersion });
});

// Start the server
const port =4000; // Update with the desired port number
app.listen(port, () => {
  console.log(`Server running on port ${port}`);
});