"""
The module is designed by team Robokit of Phystech Lyceum and team Starkit
of MIPT under mentorship of Azer Babaev 
with paticipation of junior student Matvei Ivaschenko 
and team coach Dmitrij Ustinov. 
The module is designed for creating strategy matrix of "forward" player.
Strategy is defined by direction and power of kick in selected segment of field.
Order of operation with "strategy_designer" module: 
- launch under python interpreter;
- using menu item "Load from file" choose file with strategy data. Usually 
    with name "Init_params/startegy_data.json";
- select sector for modification by click;
- operating slider "YAW" at right upper side set direction of kick;
- operating slider "POWER" set power of kick;
- click button "RECORD"
- select next sector for modification;
- save changes into file;
- quit;
There is console window at right side of program window reporting about steps of operation.
Last configuration of program is saved in the file "startegy_designer_config.json" 
which must be in the same directory as program itself.  
"""

import wx
import wx.lib.agw.rulerctrl, wx.lib.intctrl, wx.lib.masked.ipaddrctrl, wx.adv, wx.lib.floatcanvas.FCObjects, wx.lib.agw.speedmeter
import sys

import io
#import serial, serial.tools.list_ports, socket, 
import time
import random
import json
import threading
import os
#import pkg_resources.py2_warn
import math

current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/') + '/'

class RedirectText(object):
    def __init__(self,aWxTextCtrl):
        self.out = aWxTextCtrl

    def write(self,string):
        self.out.WriteText(string)

class MyPanel(wx.Panel):

    def __init__(self,parent=None):
        wx.Panel.__init__(self,parent,id=-1)
        self.parent = parent
        self.Bind(wx.EVT_PAINT, self.OnPaint)

    def OnPaint(self, evt):
        self.dc = wx.PaintDC(self)
        #dc.SetBackground(wx.Brush('#1ac500'))
        #dc.SetPen(wx.Pen('#d4d4d4'))
        self.SetClientSize(800, 600)
        self.dc.SetBrush(wx.Brush('#1ac500'))
        self.dc.DrawRectangle(0, 0, 800, 600)
        pen1 = wx.Pen('#ffffff', 10, wx.SOLID)
        pen1.SetJoin(wx.JOIN_MITER)
        self.dc.SetPen(pen1)
        self.dc.DrawRectangle(40, 40, 720, 520)
        self.dc.DrawRectangle(0, 200, 40, 200)
        self.dc.DrawRectangle(760, 200, 40, 200)
        self.dc.DrawRectangle(40, 160, 40, 280)
        self.dc.DrawRectangle(720, 160, 40, 280)
        self.dc.DrawCircle(400,300,60)
        self.dc.DrawLine(400,45,400,555)
        self.dc.DrawLine(390,300,410,300)
        self.dc.DrawLine(210,300,230,300)
        self.dc.DrawLine(220,290,220,310)
        self.dc.DrawLine(570,300,590,300)
        self.dc.DrawLine(580,290,580,310)
        pen2 = wx.Pen('#000000', 1, wx.SOLID)
        pen3 = wx.Pen('#ffff00', 1, wx.SOLID)
        self.dc.SetPen(pen2)
        for vert in range(19):
            self.dc.DrawLine(40 + vert * 40, 40, 40 + vert * 40, 560)
        for horiz in range(14):
            self.dc.DrawLine(40, 40 + horiz * 40, 760, 40 + horiz * 40)
        self.dc.DrawRectangle(38 + self.parent.activeSector[0] * 40 , 38 + self.parent.activeSector[1] * 40, 44, 44)  # activeSector
        for i in range(len(self.parent.strategy_data)):
            column = 60 + self.parent.strategy_data[i][0] * 40
            raw = 60 + self.parent.strategy_data[i][1] * 40
            arrow_length = 30 / self.parent.strategy_data[i][2]
            arrow_start_x = column - arrow_length / 2 * math.cos(self.parent.strategy_data[i][3])
            arrow_end_x = column + arrow_length / 2 * math.cos(self.parent.strategy_data[i][3])
            arrow_start_y = raw + arrow_length / 2 * math.sin(self.parent.strategy_data[i][3])
            arrow_end_y = raw - arrow_length / 2 * math.sin(self.parent.strategy_data[i][3])
            self.dc.DrawLine(arrow_start_x, arrow_start_y, arrow_end_x, arrow_end_y)
            self.dc.DrawCircle(arrow_end_x, arrow_end_y,2)
        distance = 300/ self.parent.activeSectorPower
        column = 60 + self.parent.activeSector[0] * 40
        raw = 60 + self.parent.activeSector[1] * 40
        arrow_length = 30 / self.parent.activeSectorPower
        arrow_start_x = column - arrow_length / 2 * math.cos(self.parent.activeSectorYaw)
        arrow_end_x = column + arrow_length / 2 * math.cos(self.parent.activeSectorYaw)
        arrow_end_x_inf = column + distance * math.cos(self.parent.activeSectorYaw)
        arrow_start_y = raw + arrow_length / 2 * math.sin(self.parent.activeSectorYaw)
        arrow_end_y = raw - arrow_length / 2 * math.sin(self.parent.activeSectorYaw)
        arrow_end_y_inf = raw - distance * math.sin(self.parent.activeSectorYaw)
        if arrow_end_x_inf > 800: 
            arrow_end_x_inf = 800
            arrow_end_y_inf = raw - (800- column) * math.tan(self.parent.activeSectorYaw)
        if arrow_end_x_inf < 0: 
            arrow_end_x_inf = 0
            arrow_end_y_inf = raw + column * math.tan(self.parent.activeSectorYaw)
        if arrow_end_y_inf > 600: 
            arrow_end_y_inf = 600
            arrow_end_x_inf = column - (600 - raw) / math.tan(self.parent.activeSectorYaw)
        if arrow_end_y_inf < 0: 
            arrow_end_y_inf = 0
            arrow_end_x_inf = column + raw / math.tan(self.parent.activeSectorYaw)
        self.dc.SetPen(pen3)
        self.dc.DrawLine(arrow_start_x, arrow_start_y, arrow_end_x_inf, arrow_end_y_inf)
        self.dc.DrawCircle(arrow_end_x, arrow_end_y,2)
        self.dc.DrawCircle(arrow_end_x_inf, arrow_end_y_inf,2)


class Strategy_Designer(wx.Frame):

    def __init__(self, *args, **kw):
        super(Strategy_Designer, self).__init__(*args, **kw)
        self.COLUMNS = 18
        self.RAWS = 13
        self.strategy_data = []
        for column in range(self.COLUMNS):
            for raw in range(self.RAWS):
                self.strategy_data.append([column,raw,1,0])
        self.strategy_data_file_is_loaded = False
        self.panel = wx.Panel(self)
        self.filename = 'strategy_data.json'
        self.controlslider = [0,0]
        self.controlvalue = [0,0]
        self.activeSectorYaw = 0
        self.activeSectorPower = 1
        self.activeSector = [0,0]
        self.InitUI()
        
    def InitUI(self):
        self.console_Panel = wx.Panel(self)
        self.log = wx.TextCtrl(self.console_Panel, -1, style=wx.TE_MULTILINE)
        log_box = wx.BoxSizer(wx.VERTICAL)
        log_box.Add(self.log, proportion = 1, flag=wx.EXPAND|wx.BOTTOM|wx.TOP)
        self.console_Panel.SetSizer(log_box)
        redir = RedirectText(self.log)
        sys.stdout = redir
        #sys.stderr = redir
        self.CreateMenuBar()
        self.videopanel = MyPanel(self)
        self.videopanel.SetBackgroundStyle(wx.BG_STYLE_SYSTEM)
        self.videopanel.SetBackgroundColour(wx.BLACK)
        self.videopanel.SetMinSize((800,600))

        self.yaw_control_pnl = wx.Panel(self, pos=(800, 0) , size = (130,150), style = wx.TAB_TRAVERSAL|wx.BORDER_RAISED)
        self.yaw_control_pnl.SetBackgroundColour(wx.SystemSettings.GetColour(wx.SYS_COLOUR_MENU))
        val1 = 0
        controlName0 = wx.StaticText(self.yaw_control_pnl, label='YAW', pos = (0,0), size=(70,25), style= wx.ALIGN_LEFT)
        self.controlslider[0] = wx.Slider(self.yaw_control_pnl, id = 0,   value=val1, minValue=-180, maxValue=180, pos = (0,25), size = (130,25))
        self.controlvalue[0] = wx.SpinCtrl(self.yaw_control_pnl, id = 0, value="0", pos=(70,0), size=wx.DefaultSize,
                                  style=wx.SP_WRAP, min= -180, max=180, initial=0)
        self.controlslider[0].Bind(wx.EVT_SLIDER, self.On_Slider_move)
        self.controlvalue[0].Bind(wx.EVT_SPINCTRL, self.On_SPINCTRL_change)
        self.controlvalue[0].Bind(wx.EVT_TEXT, self.On_SPINCTRL_change)

        controlName1 = wx.StaticText(self.yaw_control_pnl, label='POWER', pos = (0,100), size=(70,25), style= wx.ALIGN_LEFT)
        self.controlslider[1] = wx.Slider(self.yaw_control_pnl, id = 1,   value= 1, minValue=1, maxValue=3, pos = (0,125), size = (130,25))
        self.controlvalue[1] = wx.SpinCtrl(self.yaw_control_pnl, id = 1, value="1", pos=(70,100), size=wx.DefaultSize,
                                  style=wx.SP_WRAP, min= 1, max=3, initial=1)
        self.controlslider[1].Bind(wx.EVT_SLIDER, self.On_Slider_move)
        self.controlvalue[1].Bind(wx.EVT_SPINCTRL, self.On_SPINCTRL_change)
        self.controlvalue[1].Bind(wx.EVT_TEXT, self.On_SPINCTRL_change)

        self.quit_button =  wx.Button(self.panel, wx.ID_ANY, "Quit")
        self.save_and_exit_button = wx.Button(self.panel, wx.ID_ANY, "Save&Exit")
        self.record_button = wx.Button(self.panel, wx.ID_ANY, "Record")
        self.load_file_button = wx.Button(self.panel, wx.ID_ANY, "Load File")

        self.button_box = wx.BoxSizer(wx.HORIZONTAL)
        vbox = wx.BoxSizer(wx.VERTICAL)
        hbox1 = wx.BoxSizer(wx.HORIZONTAL)
        hbox2 = wx.BoxSizer(wx.HORIZONTAL)

        self.button_box.Add(self.quit_button, proportion=0)
        self.button_box.Add(self.save_and_exit_button, proportion=0)
        self.button_box.Add(self.load_file_button, proportion=0)
        self.button_box.Add(self.record_button, proportion=0, flag= wx.EXPAND | wx.LEFT)

        self.panel.SetSizer(self.button_box)
        self.button_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.button_sizer.Add(self.panel, proportion=2, flag=wx.EXPAND  )
        image_sizer = wx.BoxSizer(wx.HORIZONTAL)
        image_sizer.Add(self.videopanel, proportion=0)
        image_sizer.Add(self.yaw_control_pnl, proportion=0, flag=wx.ALIGN_LEFT)

        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.button_sizer, proportion=0, flag=wx.EXPAND )
        sizer.Add(image_sizer, proportion=2, flag=wx.EXPAND)
        sizer.SetMinSize((810, 680))

        whole_window = wx.BoxSizer(wx.HORIZONTAL)
        whole_window.Add(sizer, proportion = 0, flag=wx.EXPAND )
        whole_window.Add(self.console_Panel, proportion = 1, flag=wx.EXPAND|wx.BOTTOM|wx.TOP  )

        self.SetMinSize((1000, 710))
        self.CreateStatusBar()
        self.SetSizer(whole_window)

        self.SetSize((1000, 710))
        self.SetTitle('Strategy Designer')
        self.Centre()
        self.videopanel.Bind(wx.EVT_LEFT_DOWN, self.On_Left_Down_Field)
        self.Bind(wx.EVT_SLIDER, self.On_Slider_move)
        self.Bind(wx.EVT_MENU, self.On_Load_File, id = 1 )
        self.Bind(wx.EVT_MENU, self.On_Save, id = 2 )
        self.Bind(wx.EVT_MENU, self.On_Save_as, id = 3 )
        self.Bind(wx.EVT_MENU, self.On_Quit_select, id = 5 )
        self.Bind(wx.EVT_MENU, self.OnAbout, id = 21 )
        self.Bind(wx.EVT_MENU, self.Quick_Start, id = 22 )
        self.quit_button.Bind(wx.EVT_BUTTON, self.On_Quit_select)
        self.record_button.Bind(wx.EVT_BUTTON, self.On_Record )
        self.load_file_button.Bind(wx.EVT_BUTTON, self.On_Load_File)
        self.save_and_exit_button.Bind(wx.EVT_BUTTON, self.On_Save_and_Exit)

    def On_Slider_move(self, event):
        id = event.GetId()
        val = self.controlslider[id].GetValue()
        self.controlvalue[id].SetValue(val)
        if id == 0: self.activeSectorYaw =  math.radians(val) 
        if id == 1: self.activeSectorPower = val
        self.videopanel.Refresh()

    def On_SPINCTRL_change(self, event):
        id = event.GetId()
        val = self.controlvalue[id].GetValue()
        self.controlslider[id].SetValue(val)
        if id == 0: self.activeSectorYaw =  math.radians(val) 
        if id == 1: self.activeSectorPower = val
        self.videopanel.Refresh()

    def On_Left_Down_Field(self, event):
        mouse_position = event.GetPosition()
        column_position = mouse_position[0] // 40 - 1
        raw_position = mouse_position[1] // 40 - 1
        legalPosition = False
        if 0 <= column_position < 18  and 0 <= raw_position < 13 : legalPosition = True
        if legalPosition: 
            self.activeSector = [column_position, raw_position]
            print((column_position, raw_position))
        self.videopanel.Refresh()

    def Quick_Start(self, event):
        print('1. Здесь можно дать послеловательность действий оператора',
              '\n2. Это должно помочь')

    def OnAbout(self, event):
        aboutInfo = wx.adv.AboutDialogInfo()
        aboutInfo.SetName("Strategy Designer")
        aboutInfo.SetVersion('Version 1.0')
        aboutInfo.SetDescription("With this app you can tune fast and convenient\n   strategy of ball kicks depending on ball position.")
        aboutInfo.SetCopyright("(C) 2020")
        aboutInfo.SetWebSite("www.robokit.su")
        aboutInfo.AddDeveloper("Matvei Ivaschenko")
        wx.adv.AboutBox(aboutInfo)

    def On_Save(self, event):
        data = {"strategy_data": self.strategy_data}
        with open(self.filename, "w") as f:
                json.dump(data, f)

    def On_Save_as(self, event):
        save_file_dialog = wx.FileDialog(None, message="Select .json file with strategy data",
                                        wildcard = '*.json', style =wx.FD_SAVE|wx.FD_OVERWRITE_PROMPT)
        success_code = save_file_dialog.ShowModal()
        if success_code == wx.ID_OK:
            self.filename = save_file_dialog.GetPath()
            self.filename = self.filename.replace('\\', '/')
            data = {"strategy_data": self.strategy_data}
            with open(self.filename, "w") as f:
                json.dump(data, f)

    def On_Save_and_Exit(self, event):
        self.On_Save(event)
        self.On_Quit_select(event)

    def CreateMenuBar(self):
        menubar = wx.MenuBar()
        self.filem = wx.Menu()
        help = wx.Menu()

        self.filem.Append(1, '&Load from file', 'Load strategy data from file')
        self.filem.Append(2, '&Save', 'Save strategy data to loaded file')
        self.filem.Append(3, 'Save as', 'Save strategy data to new file')
        self.filem.Append(5, '&Quit', 'Quit application')

        menubar.Append(self.filem, '&File')
        menubar.Append(help, '&Help')

        help.Append(21,'About')
        help.Append(22,'Quick Start')
        self.SetMenuBar(menubar)

    def On_Quit_select(self, e):
        #with open(current_work_directory + "Threshold_Tuner_config.json", "w") as f:
        #        json.dump(self.config, f)
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__
        sys.exit(0)

    def On_Record(self, e):
        index = -1
        for i in range(len(self.strategy_data)):
            if self.strategy_data[i][0] == self.activeSector[0] and self.strategy_data[i][1] == self.activeSector[1]:
                index = i
                self.strategy_data[i] = [self.activeSector[0], self.activeSector[1], self.activeSectorPower, self.activeSectorYaw]
                break
        if index == -1:
            self.strategy_data.append([self.activeSector[0], self.activeSector[1], self.activeSectorPower, self.activeSectorYaw])
        self.videopanel.Refresh()

    def On_Load_File(self, event):
        load_file_dialog = wx.FileDialog(None, message="Select .json file with strategy data", defaultFile = self.filename, wildcard = '*.json')
        success_code = load_file_dialog.ShowModal()
        if success_code == wx.ID_OK:
            self.filename = load_file_dialog.GetPath()
            self.defaultFile = self.filename
            self.filename = self.filename.replace('\\', '/')
            with open(self.filename, "r") as f:
                loaded_Dict = json.loads(f.read())
            if loaded_Dict.get('strategy_data') != None:
                self.strategy_data_file_is_loaded = True
                self.strategy_data = loaded_Dict['strategy_data']
        print( 'stratedy_data_file_is_loaded =', self.strategy_data_file_is_loaded)
        self.videopanel.Refresh()

def main():
    app = wx.App()
    th = Strategy_Designer(None)
    th.Show()
    app.MainLoop()


if __name__ == '__main__':
    main()  

