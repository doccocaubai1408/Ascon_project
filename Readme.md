# Đây là dự án triển khai Ascon trên ESP32 


## Để thuật toán có thể hoạt động ,bạn cần làm theo hướng dẫn như sau.
 
### Chuẩn bị phần mềm.
1. Sử dụng phần mềm Arduino IDE phiên bản mới nhất.
2. Trong mục Tool/Board/Board Manager gõ tìm kiếm từ khóa: "esp32" ---> chọn tải xuống esp32 by Espressif System.
3. Trong mục File/Preferences/Setting/Additional boards manager URLs: 
    Dán Link : https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
#### Triển khai thuật toán.
1. Trong mỗi thư mục Ascon_Hash || Ascon_128 || Ascon_80PQ đều có  *2 thư mục con* và *1 file .rar || zip*
    2 thư mục con lần lượt là code triển khai thuật toán với *ngoại vi || cảm biến + benchmark* và *benchmark đơn thuần*
    1 File .zip là thư viện của thuật toán tương ứng.
       + Bạn cần giải nén nó và sao chép vào thư viện theo Arduino IDE theo đường dẫn:
        C:\Users\Admin\Documents\Arduino\libraries. 
        # tùy theo máy bạn khi cài Arduino IDE mà thư mục libraries sẽ nằm đâu đó trong máy.
2. Sau khi cài xong theo hướng dẫn,bạn vào 1 trong 2 thư mục (tùy mục đích ) để biên dịch và nạp code xuống ESP32.

