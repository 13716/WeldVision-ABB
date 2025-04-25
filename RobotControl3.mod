MODULE RobotControl
    VAR socketdev client_socket;
    VAR string received_data;
    VAR num coords{3};
    VAR robtarget target_pos;
    VAR num weld_path{100, 3};
    VAR num path_length := 0;
    PERS wobjdata my_wobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
    CONST robtarget home_pos := [[506.29,10,679.5],[0.514,0,0.857,0],[1,0,0,0],[0,0,0,0,0,0]];
    VAR bool move_success := TRUE;
    PERS bool GripperClosed := FALSE;
    CONST robtarget mid_pos:=[[350,300,800],[0.514,0,0.857,0],[1,0,0,0],[0,0,0,0,0,0]];
    PERS tooldata my_tool:=[TRUE,[[0,0,0],[1,0,0,0]],[2,[0,0,10],[1,0,0,0],0.1,0.1,0.1]];
    PERS tooldata Weldgun:=[TRUE,[[20.31,0,363.689],[1,0,0,0]],[1,[0,0,10],[1,0,0,0],0.1,0.1,0.1]];
    PROC Main()
        VAR bool keep_running := TRUE;
        VAR num i;
        CONST num WAIT_TIMEOUT := 15;

        ConfL \Off;
        ConfJ \On;
        MoveJ home_pos,v100,fine,my_tool\WObj:=my_wobj;
        TPWrite " Di chuyen den vi tri Home";

        SocketCreate client_socket;
        SocketConnect client_socket, "127.0.0.1", 5000;
        TPWrite " Ket noi thanh cong voi server Python";

        WHILE keep_running DO
            SocketSend client_socket, \Str:="READY_FOR_NEW_PATH";
            TPWrite " Dang cho phan hoi tu server...";

            received_data := "";
            SocketReceive client_socket, \Str:=received_data \Time:=WAIT_TIMEOUT;

            IF StrLen(TrimString(received_data)) = 0 THEN
                TPWrite " Timeout. Khong nhan duoc du lieu tu server.";
                MoveJ mid_pos,v100,z5,my_tool\WObj:=my_wobj;
                SocketClose client_socket;
                keep_running := FALSE;
                EXIT;
            ENDIF

            IF SocketGetStatus(client_socket) <> SOCKET_CONNECTED THEN
                TPWrite " Server ngat ket noi.";
                SocketClose client_socket;
                keep_running := FALSE;
                EXIT;
            ENDIF

            received_data := TrimString(received_data);
            TPWrite " Nhan duoc: '" + received_data + "'";

            IF received_data = "DONE" THEN
                TPWrite " Server bao hoan thanh tat ca batch.";
                MoveJ home_pos, v100,fine,my_tool\WObj:=my_wobj;
                SocketClose client_socket;
                keep_running := FALSE;
                EXIT;
            ENDIF

            IF ParseWeldPath(received_data) THEN
                FOR i FROM 1 TO path_length DO
                    coords{1} := weld_path{i,1};
                    coords{2} := weld_path{i,2};
                    coords{3} := weld_path{i,3};

                    IF BuildRobTarget(i)  THEN
                        TPWrite " Diem " + NumToStr(i,0) + ": " + NumToStr(coords{1},1) + ", " + NumToStr(coords{2},1) + ", " + NumToStr(coords{3},1);
                        SetGripper(TRUE);
                        MoveL target_pos,v100,z5,my_tool\WObj:=my_wobj;
                        SetGripper(FALSE);
                    ELSE
                        TPWrite " Loi: toa do khong hop le tai diem " + NumToStr(i, 0);
                        SocketSend client_socket, \Str:="ERROR: Coordinates out of range";
                        EXIT;
                    ENDIF
                ENDFOR
            ELSE
                TPWrite " Loi: ParseWeldPath that bai";
                SocketSend client_socket, \Str:="ERROR: Invalid path format";
                EXIT;
            ENDIF
        ENDWHILE
    ENDPROC

    PROC SetGripper(BOOL closed)
        GripperClosed := closed;
        IF closed THEN
            TPWrite " Gripper dong";
        ELSE
            TPWrite " Gripper mo";
        ENDIF
    ENDPROC

  FUNC bool BuildRobTarget(num i)
        VAR num dx; 
        VAR num dy;
        VAR num angle_deg; 
        VAR num distance;
        VAR orient tool_orient;
        distance := Sqrt(coords{1}*coords{1} + coords{2}*coords{2} + coords{3}*coords{3});
        IF Abs(coords{1}) > 700 OR Abs(coords{2}) > 700 OR coords{3} > 900 OR coords{3} < -200 THEN
            RETURN FALSE;
        ENDIF
        IF distance > 810 THEN
            RETURN FALSE;
        ENDIF

        IF i < path_length THEN
            dx := weld_path{i+1,1} - weld_path{i,1};
            dy := weld_path{i+1,2} - weld_path{i,2};
        ELSEIF i > 1 THEN
            dx := weld_path{i,1} - weld_path{i-1,1};
            dy := weld_path{i,2} - weld_path{i-1,2};
        ELSE
            dx := 1;
            dy := 0;
        ENDIF

        angle_deg := ATan2(dy, dx) * 180 / 3.1415926;
        IF angle_deg >170 THEN
        angle_deg :=180;
        tool_orient := OrientZYX(-180,23, angle_deg);
        ELSE 
        angle_deg :=-180;
        tool_orient := OrientZYX(-180,23, angle_deg);  
        ENDIF
        target_pos := [[coords{1}, coords{2}, coords{3}], tool_orient, [1,0,0,0], [0,0,0,0,0,0]];
        RETURN TRUE;
    ENDFUNC

    FUNC string TrimString(string str)
        VAR num start := 1;
        VAR num last ;
        last := StrLen(str);
        WHILE start <= last AND StrPart(str, start, 1) = " " DO
            start := start + 1;
        ENDWHILE
        WHILE last >= start AND StrPart(str, last, 1) = " " DO
            last := last - 1;
        ENDWHILE
        RETURN StrPart(str, start, last - start + 1);
    ENDFUNC

    FUNC bool ParseWeldPath(string data)
        VAR num i;
        VAR num j; 
        VAR num start_pos := 1;
        VAR num end_pos; 
        VAR num point_count := 0;
        VAR string point_str;
        VAR string temp_str;
        VAR num comma_pos;

        FOR i FROM 1 TO StrLen(data) DO
            IF StrPart(data, i, 1) = ";" THEN
                point_count := point_count + 1;
            ENDIF
        ENDFOR
        IF StrLen(data) > 0 AND StrPart(data, StrLen(data), 1) <> ";" THEN
            point_count := point_count + 1;
        ENDIF
        IF point_count > 100 THEN
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

            FOR j FROM 1 TO 3 DO
                comma_pos := StrFind(point_str, 1, ",");
                IF comma_pos = -1 AND j < 3 THEN
                    RETURN FALSE;
                ENDIF
                IF j < 3 THEN
                    temp_str := StrPart(point_str, 1, comma_pos - 1);
                    point_str := StrPart(point_str, comma_pos + 1, StrLen(point_str) - comma_pos);
                ELSE
                    temp_str := point_str;
                ENDIF
                IF NOT StrToVal(temp_str, weld_path{i,j}) THEN
                    RETURN FALSE;
                ENDIF
            ENDFOR
            path_length := path_length + 1;
        ENDFOR
        RETURN TRUE;
    ENDFUNC

ENDMODULE
