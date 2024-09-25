
void print_data()
{
  
   param = "voltageInput=" + String(voltageInput) + "&" 
             + "currentInput=" + String(currentInput) + "&" 
             + "voltageOutput=" + String(voltageOutput) + "&" 
             + "currentOutput=" + String(currentOutput) + "&" 
             + "powerInput=" + String(powerInput) + "&" 
             + "batteryPercent=" + String(batteryPercent);
  Serial.println(param);
  write_to_google_sheet(param);
  
}

void write_to_google_sheet(String params)
{
  HTTPClient http;
  String url = "https://script.google.com/macros/s/" + GOOGLE_SCRIPT_ID + "/exec?" + params;
  Serial.println("Data sent to Google Sheets");
  
  // Bắt đầu gửi dữ liệu đến Google Sheets
  http.begin(url.c_str());
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  int httpCode = http.GET();
  Serial.print("HTTP Status Code: ");
  Serial.println(httpCode);
  
  // Nhận phản hồi từ Google Sheets
  String payload;
  if (httpCode > 0)
  {
    payload = http.getString();
    Serial.println("Payload: " + payload);
  }
  
  http.end();
}

