#ifndef _wirelessServerHtml_h_
#define _wirelessServerHtml_h_


const char html_top[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>

    <head>
        <meta name="viewport", charset="UTF-8", content="width=device-width", initial-scale=1.0 user-scalable=yes></meta>
        <title>ESP32test</title>
    </head>
    <body>

)rawliteral";


String html_mid = "";


const char html_bot[] PROGMEM = R"rawliteral(
        <br>
        <br>
        <form action="/savee">
            px:
            <br><input type="text" name="px" size="5">
            <input type="submit" value="save">
        </form>
        <form action="/savee">
            ix:
            <br><input type="text" name="ix" size="5">
            <input type="submit" value="save">
        </form>
        <form action="/savee">
            dx:
            <br><input type="text" name="dx" size="5">
            <input type="submit" value="save">
        </form>

        <br>
        
        <form action="/savee">
            py:
            <br><input type="text" name="py" size="5">
            <input type="submit" value="save">
        </form>
        <form action="/savee">
            iy:
            <br><input type="text" name="iy" size="5">
            <input type="submit" value="save">
        </form>
        <form action="/savee">
            dy:
            <br><input type="text" name="dy" size="5">
            <input type="submit" value="save">
        </form>

        <br>
        
        <form action="/savee">
            pz:
            <br><input type="text" name="pz" size="5">
            <input type="submit" value="save">
        </form>
        <form action="/savee">
            iz:
            <br><input type="text" name="iz" size="5">
            <input type="submit" value="save">
        </form>
        <form action="/savee">
            dz:
            <br><input type="text" name="dz" size="5">
            <input type="submit" value="save">
        </form>
        <br>
        <br>
        
        <form action="/exit">
            exit: 
            <br><input type="text" name="exit" size="5">
            <input type="submit" value="save">
        </form>
        
          
    </body>


</html>


)rawliteral";














#endif