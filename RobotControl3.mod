MODULE RobotControl
    VAR socketdev client_socket;
    VAR string received_data;
    VAR num coords{3};
    VAR robtarget target_pos;
    VAR num weld_path{100, 3};  ! M?ng luu t?a d? du?ng hàn (t?i da 100 di?m, m?i di?m có [x, y, z])
    VAR num path_length := 0;   ! S? lu?ng di?m trong c?m du?ng hàn hi?n t?i
    PERS tooldata my_tool := [TRUE,[[0,0,0],[1,0,0,0]],[2,[0,0,10],[1,0,0,0],0.1,0.1,0.1]];
    PERS wobjdata my_wobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
    CONST robtarget home_pos := [[506.29,10,679.5],[0.514,0,0.857,0],[1,0,0,0],[0,0,0,0,0,0]];
    VAR bool move_success := TRUE;
    PERS bool GripperClosed := FALSE;
    CONST robtarget mid_pos:=[[350,300,800],[0.514,0,0.857,0],[1,0,0,0],[0,0,0,0,0,0]];

    PROC Main()
        VAR bool keep_running := TRUE;
        VAR num i;
        
        ConfL \Off;
        ConfJ \On;
        MoveJ home_pos, v50, fine, my_tool \WObj:=my_wobj;
        TPWrite "Dã di chuy?n d?n v? trí Home";

        SocketCreate client_socket;
        TPWrite "K?t n?i d?n server Python t?i 127.0.0.1:5000";
        SocketConnect client_socket, "127.0.0.1", 5000;
        TPWrite "Dã k?t n?i d?n server";

      WHILE keep_running DO
            ! G?i yêu c?u c?m du?ng hàn m?i
            TPWrite "G?i READY_FOR_NEW_PATH";
            SocketSend client_socket, \Str:="READY_FOR_NEW_PATH";
            
            ! Nh?n d? li?u du?ng hàn ho?c tín hi?u DONE
            TPWrite "Dang ch? d? li?u du?ng hàn...";
            SocketReceive client_socket, \Str:=received_data \Time:=WAIT_MAX;
            IF SocketGetStatus(client_socket) <> SOCKET_CONNECTED THEN
                TPWrite "L?i: Server dã ng?t k?t n?i";
                keep_running := FALSE;
                EXIT;
            ENDIF
            TPWrite "Nh?n du?c: '" + received_data + "'";

            ! Ki?m tra tín hi?u DONE
            IF received_data = "DONE" THEN
                TPWrite "Nh?n DONE, quá trình hàn hoàn t?t";
                 ! Quay v? v? trí Home và k?t thúc
                MoveJ home_pos, v100, fine, my_tool \WObj:=my_wobj;
                TPWrite "Quay v? v? trí Home";
                SocketClose client_socket;
                TPWrite "Dóng k?t n?i client";
                keep_running := FALSE;
                EXIT;
            ENDIF

            received_data := TrimString(received_data);
            TPWrite "Sau khi c?t g?n: '" + received_data + "'";

            IF StrLen(received_data) = 0 THEN
                TPWrite "L?i: D? li?u r?ng sau khi c?t g?n";
                SocketSend client_socket, \Str:="ERROR: Empty data";
                EXIT;
            ELSEIF ParseWeldPath(received_data) THEN
                TPWrite "Nh?n du?c c?m du?ng hàn v?i " + NumToStr(path_length, 0) + " di?m";
                FOR i FROM 1 TO path_length DO
                    coords{1} := weld_path{i, 1};
                    coords{2} := weld_path{i, 2};
                    coords{3} := weld_path{i, 3};
                    IF BuildRobTarget() THEN
                        TPWrite "Di chuy?n d?n di?m hàn " + NumToStr(i, 0) + ": " + NumToStr(coords{1}, 1) + "," + NumToStr(coords{2}, 1) + "," + NumToStr(coords{3}, 1);
                        move_success := TRUE;
                        SetGripper(TRUE);  ! Kích ho?t công c? hàn
                        WaitTime 0.5;
                        MoveL target_pos, v100, z10, my_tool \WObj:=my_wobj;
                        WaitTime 0.5;  ! Mô ph?ng th?i gian hàn
                        SetGripper(FALSE);  ! T?t công c? hàn
                        IF NOT move_success THEN
                            TPWrite "L?i: MoveL th?t b?i t?i di?m " + NumToStr(i, 0);
                            SocketSend client_socket, \Str:="ERROR: MoveL failed at point " + NumToStr(i, 0);
                            EXIT;
                        ENDIF
                    ELSE
                        TPWrite "L?i: BuildRobTarget th?t b?i t?i di?m " + NumToStr(i, 0);
                        SocketSend client_socket, \Str:="ERROR: Coordinates out of reach at point " + NumToStr(i, 0);
                        EXIT;
                    ENDIF
                ENDFOR
                TPWrite "Hoàn thành c?m du?ng hàn";
            ELSE
                TPWrite "L?i: D?nh d?ng không h?p l? trong ParseWeldPath";
                SocketSend client_socket, \Str:="ERROR: Invalid weld path format";
                EXIT;
            ENDIF
        ENDWHILE
        

        
    ERROR
        TPWrite "L?I: G?p ngo?i l? trong Main";
        TPWrite "Mã l?i: " + NumToStr(ERRNO, 0);
        IF ERRNO = ERR_SOCK_CLOSED THEN
            TPWrite "Socket dóng b?t ng?";
            SocketClose client_socket;
            RETRY;
        ELSE
            TPWrite "L?I: Thao tác th?t b?i v?i ERRNO: " + NumToStr(ERRNO, 0);
            SocketSend client_socket, \Str:="ERROR: Operation failed with ERRNO: " + NumToStr(ERRNO, 0);
            move_success := FALSE;
            RETRY;
        ENDIF
    ENDPROC

    PROC SetGripper(BOOL closed)
        GripperClosed := closed;
        IF closed THEN
            TPWrite "Kích ho?t công c? hàn";
        ELSE
            TPWrite "T?t công c? hàn";
        ENDIF
    ENDPROC

    FUNC string TrimString(string str)
        VAR num start := 1;
        VAR num last;
        VAR string ch;
        VAR bool found_non_space := FALSE;
        
        last := StrLen(str);
        
        IF last = 0 THEN
            RETURN "";
        ENDIF
        
        WHILE start <= last AND found_non_space = FALSE DO
            ch := StrPart(str, start, 1);
            IF (ch = " ") OR (ch = "\0D") OR (ch = "\0A") OR (ch = "\09") THEN
                start := start + 1;
            ELSE
                found_non_space := TRUE;
            ENDIF
        ENDWHILE
        
        IF start > last THEN
            RETURN "";
        ENDIF
        
        found_non_space := FALSE;
        WHILE last >= start AND found_non_space = FALSE DO
            ch := StrPart(str, last, 1);
            IF (ch = " ") OR (ch = "\0D") OR (ch = "\0A") OR (ch = "\09") THEN
                last := last - 1;
            ELSE
                found_non_space := TRUE;
            ENDIF
        ENDWHILE
        
        IF last < start THEN
            RETURN "";
        ENDIF
        
        RETURN StrPart(str, start, last - start + 1);
        
    ERROR
        TPWrite "L?I trong TrimString: G?p ngo?i l?";
        RETURN "";
    ENDFUNC

    FUNC bool ParseWeldPath(string data)
        VAR num i;
        VAR num j;
        VAR string point_str;
        VAR num start_pos := 1;
        VAR num end_pos;
        VAR num point_count := 0;
        VAR string temp_str;
        VAR num comma_pos;
        
        ! D?m s? di?m (m?i di?m phân tách b?ng d?u ch?m ph?y)
        FOR i FROM 1 TO StrLen(data) DO
            IF StrPart(data, i, 1) = ";" THEN
                point_count := point_count + 1;
            ENDIF
        ENDFOR
        IF StrLen(data) > 0 AND StrPart(data, StrLen(data), 1) <> ";" THEN
            point_count := point_count + 1;  ! Tính di?m cu?i n?u không có d?u ch?m ph?y ? cu?i
        ENDIF
        
        IF point_count > 100 THEN
            TPWrite "L?i: Quá nhi?u di?m trong du?ng hàn (t?i da 100)";
            RETURN FALSE;
        ENDIF
        
        path_length := 0;
        FOR i FROM 1 TO point_count DO
            end_pos := StrFind(data, start_pos, ";");
            IF end_pos = -1 THEN
                point_str := StrPart(data, start_pos, StrLen(data) - start_pos + 1);
            ELSE
                point_str := StrPart(data, start_pos, end_pos - start_pos);
                start_pos := end_pos + 1;
            ENDIF
            
            ! Phân tích t?a d? [x,y,z] trong point_str
            FOR j FROM 1 TO 3 DO
                comma_pos := StrFind(point_str, 1, ",");
                IF comma_pos = -1 AND j < 3 THEN
                    TPWrite "L?i: Thi?u d?u ph?y trong di?m " + NumToStr(i, 0);
                    RETURN FALSE;
                ENDIF
                IF j < 3 THEN
                    temp_str := StrPart(point_str, 1, comma_pos - 1);
                    point_str := StrPart(point_str, comma_pos + 1, StrLen(point_str) - comma_pos);
                ELSE
                    temp_str := point_str;
                ENDIF
                
                IF StrLen(temp_str) = 0 THEN
                    TPWrite "L?i: T?a d? r?ng trong di?m " + NumToStr(i, 0);
                    RETURN FALSE;
                ENDIF
                
                IF NOT StrToVal(temp_str, weld_path{i, j}) THEN
                    TPWrite "L?i: S? không h?p l? trong di?m " + NumToStr(i, 0) + ": '" + temp_str + "'";
                    RETURN FALSE;
                ENDIF
            ENDFOR
            path_length := path_length + 1;
        ENDFOR
        
        TPWrite "Dã phân tích du?ng hàn v?i " + NumToStr(path_length, 0) + " di?m";
        RETURN TRUE;
    ENDFUNC

    FUNC bool BuildRobTarget()
        VAR num distance;
        distance := Sqrt(coords{1}*coords{1} + coords{2}*coords{2} + coords{3}*coords{3});
        
        IF Abs(coords{1}) > 700 OR Abs(coords{2}) > 700 OR coords{3} > 900 OR coords{3} < -200 THEN
            TPWrite "L?i: T?a d? ngoài không gian làm vi?c (x:±700, y:±700, z:-200 d?n 900)";
            RETURN FALSE;
        ENDIF
        
        IF distance > 810 THEN
            TPWrite "L?i: T?a d? ngoài t?m v?i c?a IRB140: " + NumToStr(distance, 1) + " mm";
            RETURN FALSE;
        ENDIF
        
        target_pos := [[coords{1}, coords{2}, coords{3}], [0.514, 0, 0.857, 0], [1, 0, 0, 0], [0, 0, 0, 0, 0, 0]];
        RETURN TRUE;
        
    ERROR
        TPWrite "L?I trong BuildRobTarget: " + NumToStr(ERRNO, 0);
        RETURN FALSE;
    ENDFUNC
ENDMODULE