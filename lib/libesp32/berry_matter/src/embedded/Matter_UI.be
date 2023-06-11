#
# Matter_UI.be - WebUI for Matter configuration in Tasmota
#
# Copyright (C) 2023  Stephan Hadinger & Theo Arends
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

#######################################################################
# Matter Web UI
#
#######################################################################

import matter

#@ solidify:Matter_UI,weak

#################################################################################
# Partition_wizard_UI
#
# WebUI for the partition manager
#################################################################################
class Matter_UI
  static var _CLASSES_TYPES = "|relay|light0|light1|light2|light3|shutter|shutter+tilt"
                              "|temperature|pressure|illuminance|humidity|occupancy|onoff"
  # static var _CLASSES_HTTP  = "-http"
  static var _CLASSES_TYPES2= "|http_relay|http_light0|http_light1|http_light2|http_light3"
                              "|http_temperature|http_pressure|http_illuminance|http_humidity"
                              "|http_occupancy"
  var device

  # ####################################################################################################
  # Static function to compare two maps (shallow compare)
  # return true if equal
  static def equal_map(a, b)
    # all items of a are in b
    for k: a.keys()
      if !b.contains(k)   return false  end
      if b[k] != a[k]     return false  end
    end
    for k: b.keys()
      if !a.contains(k)   return false  end
      if b[k] != a[k]     return false  end
    end
    return true
  end

  # ####################################################################################################
  # Constructor
  def init(device)
    self.device = device
    tasmota.add_driver(self)
  end

  # ####################################################################################################
  # Init web handlers
  # ####################################################################################################
  # Displays a "Autoconf" button on the configuration page
  def web_add_config_button()
    import webserver
    # webserver.content_send("<p><form id=ac action='matterc' style='display: block;' method='get'><button>Configure Matter</button></form></p>")
    webserver.content_send("<p><form id=ac action='matterc' style='display: block;' method='get'><button>")
    webserver.content_send(matter._LOGO)
    webserver.content_send(" Configure Matter</button></form></p>")
  end

  #- ---------------------------------------------------------------------- -#
  #- Is Matter enabled?
  #- ---------------------------------------------------------------------- -#
  def matter_enabled()
    return bool(tasmota.get_option(matter.MATTER_OPTION))
  end

  #- ---------------------------------------------------------------------- -#
  #- Show commissioning information and QR Code
  #
  # Returns true if Matter is enabled
  #- ---------------------------------------------------------------------- -#
  def show_enable()
    import webserver
    import string
    var matter_enabled = self.matter_enabled

    webserver.content_send("<fieldset><legend><b>&nbsp;Matter &nbsp;</b></legend><p></p>"
                           "<p style='width:320px;'>Check the <a href='https://tasmota.github.io/docs/Matter/' target='_blank'>Matter documentation</a>.</p>"
                           "<form action='/matterc' method='post'>")

    # checkbox for Matter enable
    webserver.content_send(string.format("<p><input id='menable' type='checkbox' name='menable' %s>", self.matter_enabled() ? "checked" : ""))
    webserver.content_send("<label for='menable'><b>Matter enable</b></label></p>")

    if self.matter_enabled()
      # checkbox for Matter commissioning
      webserver.content_send(string.format("<p><input id='comm' type='checkbox' name='comm' %s>", self.device.commissioning_open != nil ? "checked" : ""))
      webserver.content_send("<label for='comm'><b>Commissioning open</b></label></p>")
    end

    webserver.content_send("<p></p><button name='save' class='button bgrn'>Save</button></form></p>"
                           "<p></p></fieldset><p></p>")
  end

  #- ---------------------------------------------------------------------- -#
  #- Show QR Code
  #- ---------------------------------------------------------------------- -#
  def show_qrcode(qr_text)
    import webserver
    # QRCode via UTF8
    var empty = " "
    var lowhalf = "\342\226\204"
    var uphalf = "\342\226\200"
    var full = "\342\226\210"

    var qr = matter.QRCode.encode_str(qr_text)
    var bitmap = qr['bitmap']
    var sz = qr['size']

    webserver.content_send('<style>.qr{font-family:monospace; margin:0; padding:0; white-space:pre; font-size:18px; color:#fff; line-height:100%;}</style>')


    webserver.content_send("<div style='transform:scale(.8,1); display:inline-block;'>")

    var s = "<div class='qr'>"
    webserver.content_send(s)
    s = ""
    for i: 0 .. sz + 1    s += lowhalf  end
    s += "</div>"
    webserver.content_send(s)
    for i: 0 .. (sz+1)/2 - 1
      s = "<div class='qr' style='background-color:#000;'>" + full
      for j: 0 .. sz - 1
        var high = (bitmap[i*2][j] == " ")
        var low = (i*2+1 < sz) ? (bitmap[i*2+1][j] == " ") : true     # default to true for bottom margin if size is odd
        s += high ? (low ? full : uphalf) : (low ? lowhalf : empty)
      end
      s += full
      s += "</div>"
      webserver.content_send(s)
    end
    # webserver.content_send("</div>")
    if sz % 2 == 0
      s = "<div class='qr' style='background-color:#000;'>"
      for i: 0 .. sz + 1    s += uphalf  end
      s += "/<div>"
      webserver.content_send(s)
    end

    webserver.content_send("</div>")
  end

  #- ---------------------------------------------------------------------- -#
  #- Show commissioning information and QR Code
  #- ---------------------------------------------------------------------- -#
  def show_commissioning_info()
    import webserver
    import string

    var seconds_left = (self.device.commissioning_open - tasmota.millis()) / 1000
    if seconds_left < 0   seconds_left = 0 end
    var min_left = (seconds_left + 30) / 60

    webserver.content_send(string.format("<fieldset><legend><b>&nbsp;Commissioning open for %i min&nbsp;</b></legend><p></p>", min_left))

    var pairing_code = self.device.compute_manual_pairing_code()
    webserver.content_send(string.format("<p>Manual pairing code:<br><b>%s-%s-%s</b></p><hr>", pairing_code[0..3], pairing_code[4..6], pairing_code[7..]))

    webserver.content_send("<div><center>")
    var qr_text = self.device.compute_qrcode_content()
    self.show_qrcode(qr_text)
    webserver.content_send(string.format("<p> %s</p>", qr_text))
    webserver.content_send("</div><p></p></fieldset><p></p>")

  end

  #- ---------------------------------------------------------------------- -#
  #- Show Passcode / discriminator form
  #- ---------------------------------------------------------------------- -#
  def show_passcode_form()
    import webserver
    import string

    webserver.content_send("<fieldset><legend><b>&nbsp;Matter Advanced Configuration&nbsp;</b></legend><p></p>")
    #
    webserver.content_send("<form action='/matterc' method='post' onsubmit='return confirm(\"This will cause a restart.\");'>"
                           "<p>Passcode:</p>")
    webserver.content_send(string.format("<input type='number' min='1' max='99999998' name='passcode' value='%i'>", self.device.root_passcode))
    webserver.content_send("<p>Distinguish id:</p>")
    webserver.content_send(string.format("<input type='number' min='0' max='4095' name='discriminator' value='%i'>", self.device.root_discriminator))
    webserver.content_send(string.format("<p><input type='checkbox' name='ipv4'%s>IPv4 only</p>", self.device.ipv4only ? " checked" : ""))
    webserver.content_send("<p></p><button name='passcode' class='button bgrn'>Change</button></form></p>"
                           "<p></p></fieldset><p></p>")

  end

  #- ---------------------------------------------------------------------- -#
  #- Show commissioning information and QR Code
  #- ---------------------------------------------------------------------- -#
  def show_fabric_info()
    import webserver
    import string

    webserver.content_send("<fieldset><legend><b>&nbsp;Fabrics&nbsp;</b></legend><p></p>"
                           "<p>Associated fabrics:</p>")

    if size(self.device.sessions.sessions) == 0
      webserver.content_send("<p><b>None</b></p>")
    else
      var first = true
      for f : self.device.sessions.fabrics.persistables()
        if !first     webserver.content_send("<hr>") end
        first = false

        var label = f.fabric_label
        if !label   label = "<No label>"    end
        label = webserver.html_escape(label)      # protect against HTML injection
        
        webserver.content_send(string.format("<fieldset><legend><b>&nbsp;#%i %s</b> (%s)&nbsp;</legend><p></p>", f.get_fabric_index(), label, f.get_admin_vendor_name()))

        var fabric_rev = f.get_fabric_id().copy().reverse()
        var deviceid_rev = f.get_device_id().copy().reverse()
        webserver.content_send(string.format("Fabric: %s<br>", fabric_rev.tohex()))
        webserver.content_send(string.format("Device: %s<br>&nbsp;", deviceid_rev.tohex()))

        webserver.content_send("<form action='/matterc' method='post' onsubmit='return confirm(\"Are you sure?\");'>")
        webserver.content_send(string.format("<input name='del_fabric' type='hidden' value='%i'>", f.get_fabric_index()))
        webserver.content_send("<button name='del' class='button bgrn'>Delete Fabric</button></form></p>")

        webserver.content_send("<p></p></fieldset><p></p>")
      end
    end

    webserver.content_send("<p></p></fieldset><p></p>")

  end

  #- ---------------------------------------------------------------------- -#
  #- Show plugins configuration
  #- ---------------------------------------------------------------------- -#
  def show_plugins_configuration()
    import webserver
    import string
    import introspect

    webserver.content_send("<fieldset><legend><b>&nbsp;Current Configuration&nbsp;</b></legend><p></p>")

    # webserver.content_send("<p></p><form style='display: block;' action='matteradd' method='get'><button class='button bgrn' name=''>Create new endpoint</button></form>")
    # webserver.content_send("<div style='display: block;'></div><hr>")

    webserver.content_send("<form action='/matterc' method='post'"
                           "<p><b>Local sensors and devices</b></p>"
                           "<table style='width:100%'>"
                           "<tr><td width='25'>#</td><td width='115'>Type</td><td>Parameter</td><td width='15'></td></tr>")

    # display one line per plug-in
    self.device.plugins_config.remove("0")      # remove any leftover from ancient configuration
    var endpoints = self.device.k2l_num(self.device.plugins_config)
    var i = 0
    var found = false
  
    while i < size(endpoints)
      var ep = endpoints[i]
      var conf = self.device.plugins_config.find(str(ep))
      var typ = conf.find('type')
      if !typ   i += 1   continue    end

      # skip any remote class
      if string.find(typ, "http_") == 0   i += 1   continue    end

      var cl = self.device.plugins_classes.find(typ)
      var arg = ""
      if cl != nil
        arg = cl.ui_conf_to_string(cl, conf)
      end

      found = true
      webserver.content_send(string.format("<tr><td><font size='-1'><b>%i</b></font></td>", ep))

      webserver.content_send(string.format("<td><font size='-1'><b>%s</b></font></td>", self.plugin_name(conf.find('type', ''))))
      webserver.content_send(string.format("<td><font size='-1'><input type='text' name='arg%i' minlength='0' size='8' value='%s'></font></td>",
                             ep, webserver.html_escape(arg)))
      webserver.content_send(string.format("<td style='text-align:center;'><button name='del%i' "
                                           "style='background:none;border:none;line-height:1;'"
                                           " onclick=\"return confirm('Confirm removing endpoint')\""
                                           ">"
                                           "&#128293;</button></td></tr>", ep))
      i += 1
    end
    webserver.content_send("</table>")

    # if array is empty, still display <none>
    if !found
      webserver.content_send("<p>&lt;none&gt;</p>")
    end

    # add an empty line for adding a configuration
    # webserver.content_send(string.format("<tr><td><input type='text' name='ep%03i' maxlength='4' size='3' pattern='[0-9]{1,4}' value=''></td>", i))
    # webserver.content_send(string.format("<td><select name='pi%03i'>", i))
    # self.plugin_option('', self._CLASSES_TYPES, self._CLASSES_TYPES2)
    # webserver.content_send(string.format("</select></td>"))
    # webserver.content_send(string.format("<td><font size='-1'><input type='text' name='arg%03i' minlength='0' size='8' value=''></font></td></td></td></tr>", i))


    # remote devices

    webserver.content_send("<p></p>")

    # iterate on each remote device

    var remotes = []
    for conf: self.device.plugins_config
      var url = conf.find("url")
      if url != nil
        remotes.push(url)
      end
    end
    self.device.sort_distinct(remotes)
    tasmota.log("MTR: remotes: "+str(remotes), 3)

    for remote: remotes

      webserver.content_send(string.format("&#x1F517; <a target='_blank' href=\"http://%s/\">%s</a>", webserver.html_escape(remote), webserver.html_escape(remote)))
      webserver.content_send("<table style='width:100%'>")
      webserver.content_send("<tr><td width='25'></td><td width='115'></td><td></td><td width='15'></td></tr>")

      found = false
      i = 0
      while i < size(endpoints)
        var ep = endpoints[i]
        var conf = self.device.plugins_config.find(str(ep))
        var typ = conf.find('type')
        if !typ   i += 1   continue    end

        # skip any non-remote class
        if string.find(typ, "http_") != 0   i += 1   continue    end
        # check if it's the right remote
        if conf.find("url") != remote   i += 1   continue    end

        var cl = self.device.plugins_classes.find(typ)
        var arg = ""
        if cl != nil
          arg = cl.ui_conf_to_string(cl, conf)
        end

        found = true
        webserver.content_send(string.format("<tr><td width='22'><font size='-1'><b>%i</b></font></td>", ep))

        webserver.content_send(string.format("<td width='115'><font size='-1'><b>%s</b></select></font></td>", self.plugin_name(conf.find('type', ''))))
        webserver.content_send(string.format("<td><font size='-1'><input type='text' name='arg%i' minlength='0' size='8' value='%s'></font></td>",
                              ep, webserver.html_escape(arg)))
        webserver.content_send(string.format("<td width='15' style='text-align:center;'><button name='del%i' "
                                            "style='background:none;border:none;line-height:1;'"
                                            " onclick=\"return confirm('Confirm removing endpoint')\""
                                            ">"
                                            "&#128293;</button></td></tr>", ep))
        i += 1
      end
      webserver.content_send("</table><p></p>")

    end # for remote: self.device.get_remotes_list()
    


    if !found
      webserver.content_send("<p>&lt;none&gt;</p>")
    end

    webserver.content_send("<button name='config' class='button bgrn'>"
                           "Change configuration</button></form>")

    # Add new endpoint section
    webserver.content_send("<hr><p><b>Add local sensor or device</b></p>"
                           "<form action='/matterc' method='post'>"
                           "<table style='width:100%'>"
                           "<tr><td width='145'>Type</td><td>Parameter</td></tr>")

    webserver.content_send("<tr><td><font size='-1'><select name='pi'>")
    self.plugin_option('', self._CLASSES_TYPES)
    webserver.content_send("</select></font></td>")
    webserver.content_send("<td><font size='-1'><input type='text' name='arg' minlength='0' size='8' value=''></font></td>"
                           "</tr></table>")
    
    webserver.content_send("<div style='display: block;'></div>")
    webserver.content_send("<button name='addep' class='button bgrn'"
                           ">Create new endpoint</button></form>")

    # Add remote endpoint
    webserver.content_send("<hr><p><b>Add Remote Tasmota</b></p>"
                           "<form action='/matteradd' method='get'>"
                           "<table style='width:100%'>")
    webserver.content_send("<tr><td width='30'><font size='-1'><b>http://</b></font></td><td><input type='text' name='url' minlength='0' size='8' value='' required placeholder='IP or domain'></td><td width='10'><font size='-1'><b>/</b></font></td></tr>"
                           "</tr></table>")
    
    webserver.content_send("<div style='display: block;'></div>")
    webserver.content_send("<button class='button bgrn'>"
                           "Auto-configure remote Tasmota</button></form><hr>")
    
    # button "Reset and Auto-discover"
    webserver.content_send("<form action='/matterc' method='post'"
                           "onsubmit='return confirm(\"This will RESET the configuration to the default. You will need to associate again.\");'>"
                           "<button name='auto' class='button bred'>Reset and Auto-discover</button><p></p></form>")
    
    webserver.content_send("<p></p></fieldset>")

  end

  #- ---------------------------------------------------------------------- -#
  #- Show pretty name for plugin class
  #- ---------------------------------------------------------------------- -#
  def plugin_name(cur, *class_list)
    if cur == ''  return ''  end
    return self.device.get_plugin_class_displayname(cur)
  end

  #- ---------------------------------------------------------------------- -#
  #- Show all possible classes for plugin
  #- ---------------------------------------------------------------------- -#
  def plugin_option(cur, *class_list)
    import webserver
    import string
    var class_types = []
    for cl: class_list
      class_types += string.split(cl, '|')
    end
    
    var i = 0
    while i < size(class_types)
      var typ = class_types[i]
      if typ == ''
        webserver.content_send("<option value=''></option>")
      elif typ == '-http'
        webserver.content_send("<option value='' disabled>--- Tasmota Remote ---</option>")
      else
        var nam = self.device.get_plugin_class_displayname(typ)
        webserver.content_send(string.format("<option value='%s'%s>%s</option>", typ, (typ == cur) ? " selected" : "", nam))
      end
      i += 1
    end
  end


  #######################################################################
  # Display the advanced configuration page
  #######################################################################
  def page_part_mgr_adv()
    import webserver
    import string

    if !webserver.check_privileged_access() return nil end

    webserver.content_start("Matter Advanced Configuration")           #- title of the web page -#
    webserver.content_send_style()                  #- send standard Tasmota styles -#

    if self.matter_enabled()
      self.show_passcode_form()
      self.show_fabric_info()
    end
    webserver.content_button(webserver.BUTTON_CONFIGURATION)
    webserver.content_stop()                        #- end of web page -#
  end


  #######################################################################
  # Display the complete page
  #######################################################################
  def page_part_mgr()
    import webserver
    import string

    if !webserver.check_privileged_access() return nil end

    webserver.content_start("Matter")           #- title of the web page -#
    webserver.content_send_style()                  #- send standard Tasmota styles -#

    self.show_enable()
    if self.matter_enabled()
      self.show_plugins_configuration()
    end

    webserver.content_send("<div style='display: block;'></div>")
    webserver.content_send("<p></p><form id='butmat' style='display: block;' action='mattera' method='get'><button name=''>Advanced Configuration</button></form>")

    webserver.content_button(webserver.BUTTON_CONFIGURATION)
    webserver.content_stop()                        #- end of web page -#
  end

  #---------------------------------------------------------------------- -#
  # Generate configuration map from Status 8 and Status 11
  #
  # Returns a list of maps: [ {"type":"temperature", "filter":"ESP32#Temperature"} ]
  #---------------------------------------------------------------------- -#
  def generate_config_from_status(status8, status11)
    var config_list = []

    # count `Power` and `Power<x>`
    var power_cnt = 0
    if status11.contains("POWER")
      power_cnt = 1
    else
      var idx = 1
      while true
        if status11.contains("POWER" + str(idx))
          power_cnt = idx
          idx += 1
        else
          break
        end
      end
    end
    # Now `power_cnt` contains the number of Relays including light

    # detect lights
    var light1, light2, light3    # contains a relay number of nil
    if status11.contains("HSBColor")
      light3 = power_cnt
      power_cnt -= 1
    elif status11.contains("CT")
      light2 =  power_cnt
      power_cnt -= 1
    elif status11.contains("Dimmer")
      light1 =  power_cnt
      power_cnt -= 1
    end

    # rest is relays
    for i: 1..power_cnt
      config_list.push({'type': 'light0', 'relay': i})
    end

    # show lights
    if light1 != nil
      config_list.push({'type': 'light1', 'relay': light1})
    end
    if light2 != nil
      config_list.push({'type': 'light2', 'relay': light2})
    end
    if light3 != nil
      config_list.push({'type': 'light3', 'relay': light3})
    end


    # detect sensors
    config_list += self.device.autoconf_sensors_list(status8)

    return config_list
  end

  #- ---------------------------------------------------------------------- -#
  #- Probe remote device
  #- ---------------------------------------------------------------------- -#
  def show_remote_autoconf(url)
    import webserver
    import string
    import json

    if url == ''  return end
    var timeout = matter.Plugin_Bridge_HTTP.PROBE_TIMEOUT
    var http_remote = matter.HTTP_remote(url, timeout)
    # Status 8
    var status8 = http_remote.call_sync('Status 8', timeout)
    if status8 != nil   status8 = json.load(status8)                end
    if status8 != nil   status8 = status8.find('StatusSNS')         end
    # Status 11
    var status11
    if status8 != nil
      status11 = http_remote.call_sync('Status 11', timeout)
      if status11 != nil   status11 = json.load(status11)           end
      if status11 != nil   status11 = status11.find('StatusSTS')     end
    end
    
    if status8 != nil && status11 != nil
      tasmota.log(string.format("MTR: probed '%s' status8=%s satus11=%s", url, str(status8), str(status11)), 3)

      var config_list = self.generate_config_from_status(status8, status11)

      webserver.content_send("<fieldset><legend><b>&nbsp;Matter Remote Device&nbsp;</b></legend><p></p>"
                             "<p><b>Add Remote sensor or device</b></p>")

      webserver.content_send(string.format("<p>&#x1F517; <a target='_blank' href=\"http://%s/\">%s</a></p>", webserver.html_escape(url), webserver.html_escape(url)))

      # Add new endpoint section
      webserver.content_send("<form action='/matterc' method='post'>"
                             "<table style='width:100%'>"
                             "<tr><td width='145'>Type</td><td>Parameter</td></tr>")

      webserver.content_send(string.format("<input name='url' type='hidden' value='%s'>", webserver.html_escape(url)))

      var i = 0
      while i < size(config_list)
        var config = config_list[i]
        var typ = config.find('type', '')
        if typ != ''     typ = "http_" + typ     end

        var cl = self.device.plugins_classes.find(typ)
        var arg = ""
        if cl != nil
          arg = cl.ui_conf_to_string(cl, config)
        end

        webserver.content_send(string.format("<tr><td><font size='-1'><select name='pi%i'>", i))
        self.plugin_option(typ, self._CLASSES_TYPES2)
        webserver.content_send("</select></font></td>"
                               "<td><font size='-1'>")
        webserver.content_send(string.format("<input type='text' name='arg%i' minlength='0' size='8' value='%s'>",
                               i, webserver.html_escape(arg)))
        webserver.content_send("</font></td></tr>")
        i += 1
      end
      # empty line for new endpoint
      webserver.content_send(string.format("<tr><td><font size='-1'><select name='pi%i'>", i))
      self.plugin_option('', self._CLASSES_TYPES2)
      webserver.content_send("</select></font></td>"
                             "<td><font size='-1'>")
      webserver.content_send(string.format("<input type='text' name='arg%i' minlength='0' size='8' value='%s'>",
                             i, ''))
      webserver.content_send("</font></td></tr>")

      # end of table
      webserver.content_send("</table>")
      
      webserver.content_send("<div style='display: block;'></div>")
      webserver.content_send("<button name='addrem' class='button bgrn'>"
                            "Add endpoints</button></form>")

      webserver.content_send("</form></fieldset><p></p>")

    else
      webserver.content_send(string.format("<p><b>Unable to connect to '%s'</b></p>", webserver.html_escape(url)))
    end


  end

  #######################################################################
  # Display the page for adding a new endpoint
  #######################################################################
  def page_part_mgr_add()
    import webserver
    import string

    if !webserver.check_privileged_access() return nil end

    webserver.content_start("Matter Create new endpoint")           #- title of the web page -#
    webserver.content_send_style()                  #- send standard Tasmota styles -#

    var url = webserver.arg("url")
    if self.matter_enabled()
      self.show_remote_autoconf(url)
    end
    webserver.content_button(webserver.BUTTON_CONFIGURATION)
    webserver.content_stop()                        #- end of web page -#
  end


  #######################################################################
  # Web Controller, called by POST to `/matterc`
  #######################################################################
  def page_part_ctl()
    import webserver
    if !webserver.check_privileged_access() return nil end

    import string
    import partition_core
    import persist

    var error

    try

      # debug information about parameters
      for i:0..webserver.arg_size()-1
        tasmota.log(string.format("MTR: Arg%i '%s' = '%s'", i, webserver.arg_name(i), webserver.arg(i)))
      end

      #---------------------------------------------------------------------#
      # Change Passcode and/or Passcode
      #---------------------------------------------------------------------#
      if webserver.has_arg("passcode") || webserver.has_arg("discriminator")
        tasmota.log(string.format("MTR: /matterc received '%s' command", 'passcode'), 3)
        if webserver.has_arg("passcode")
          self.device.root_passcode = int(webserver.arg("passcode"))
        end
        if webserver.has_arg("discriminator")
          self.device.root_discriminator = int(webserver.arg("discriminator"))
        end
        self.device.ipv4only = webserver.arg("ipv4") == 'on'
        self.device.save_param()

        #- and force restart -#
        webserver.redirect("/?rst=")

      elif webserver.has_arg("save")
        var matter_enabled_requested = webserver.has_arg("menable")
        var matter_commissioning_requested = webserver.has_arg("comm")

        if matter_enabled_requested != self.matter_enabled()
          if matter_enabled_requested
            tasmota.log(string.format("MTR: /matterc received '%s' command", 'enable'), 3)
            tasmota.cmd("SetOption" + str(matter.MATTER_OPTION) + " 1")
          else
            tasmota.log(string.format("MTR: /matterc received '%s' command", 'disable'), 3)
            tasmota.cmd("SetOption" + str(matter.MATTER_OPTION) + " 0")
          end
          #- and force restart -#
          webserver.redirect("/?rst=")
        elif matter_commissioning_requested != (self.device.commissioning_open != nil)
          if matter_commissioning_requested
            self.device.start_root_basic_commissioning()
          else
            self.device.stop_basic_commissioning()
          end
        
          #- and force restart -#
          webserver.redirect("/")
        else
          webserver.redirect("/")
        end

      #---------------------------------------------------------------------#
      # Delete Fabric
      #---------------------------------------------------------------------#
      elif webserver.has_arg("del_fabric")
        tasmota.log(string.format("MTR: /matterc received '%s' command", 'del_fabric'), 3)
        var del_fabric = int(webserver.arg("del_fabric"))
        var idx = 0
        var fabrics = self.device.sessions.fabrics
        while idx < size(fabrics)
          if fabrics[idx].get_fabric_index() == del_fabric
            self.device.remove_fabric(fabrics[idx])
            break
          else
            idx += 1
          end
        end
        #- reload same page -#
        webserver.redirect("/matterc?")

      #---------------------------------------------------------------------#
      # Reset to default auto-configuration
      #---------------------------------------------------------------------#
      elif webserver.has_arg("auto")
        tasmota.log(string.format("MTR: /matterc received '%s' command", 'auto'), 3)
        self.device.plugins_persist = false
        self.device.save_param()
        #- and force restart -#
        webserver.redirect("/?rst=")

      #---------------------------------------------------------------------#
      # Apply new configuration template
      #---------------------------------------------------------------------#
      elif webserver.has_arg("config")
        tasmota.log(string.format("MTR: /matterc received '%s' command", 'config'), 3)
        var needs_saving = false
        # iterate by endpoint number
        for i:0..webserver.arg_size()-1
          var arg_name = webserver.arg_name(i)
          if string.find(arg_name, "arg") == 0
            var arg_ep = int(arg_name[3..])         # target endpoint as int
            var arg = webserver.arg(i)              # text value

            var conf_ep = self.device.plugins_config.find(str(arg_ep))    # find the corresponding configuration map

            if conf_ep != nil     # found
              var typ_class = self.device.plugins_classes.find(conf_ep.find('type', ''))
              if typ_class != nil
                tasmota.log(string.format("MTR: ep=%i arg=%s", arg_ep, arg), 3)
                # compute the actual value
                var prev_arg = typ_class.ui_conf_to_string(typ_class, conf_ep)
                var changed = (prev_arg != arg)
                tasmota.log(string.format("MTR: ep=%i prev_arg='%s' arg='%s' %s", arg_ep, prev_arg, arg, prev_arg != arg ? "changed" : ""), 3)

                if changed
                  needs_saving = true
                  typ_class.ui_string_to_conf(typ_class, conf_ep, arg)
                  var pl = self.device.find_plugin_by_endpoint(arg_ep)
                  if pl
                    tasmota.log(string.format("MTR: apply conf '%s' (%i) to %s", conf_ep, arg_ep, pl), 3)
                    pl.parse_configuration(conf_ep)
                  end
                end

              end
            else            
              tasmota.log(string.format("MTR: ep=%i not found", arg_ep), 3)
            end
          end
        end

        tasmota.log(string.format("MTR: config = %s", str(self.device.plugins_config)), 3)

        if error
          tasmota.log(string.format("MTR: config error = %s", error), 3)
        else
          if needs_saving || !self.device.plugins_persist
            self.device.plugins_persist = true
            self.device.save_param()
          end
          #- and reload -#
          webserver.redirect("/cn?")
        end
      #---------------------------------------------------------------------#
      # Add new endpoint for local sensor or device
      #---------------------------------------------------------------------#
      elif webserver.has_arg("addep")
        var typ = webserver.arg('pi')
        var arg = webserver.arg('arg')
        tasmota.log(string.format("MTR: add endpoint typ='%s' arg='%s'", typ, arg), 3)

        # check if type exists
        var typ_class = self.device.plugins_classes.find(typ)
        if typ_class != nil
          var config = {}
          typ_class.ui_string_to_conf(typ_class, config, arg)
          self.device.bridge_add_endpoint(typ, config)
        end
        #- and reload -#
        webserver.redirect("/matterc?")

      #---------------------------------------------------------------------#
      # Add new endpoint for local sensor or device
      #---------------------------------------------------------------------#
      elif webserver.has_arg("addrem")
        var url = webserver.arg('url')
        if url == nil || url == ''    raise "value_error", "url shouldn't be null"  end

        # iterate by id
        var idx = 0
        var idx_str = str(idx)
        while webserver.has_arg('pi'+idx_str)
          var typ = webserver.arg('pi'+idx_str)
          var arg = webserver.arg('arg'+idx_str)

          if typ != ''
            # check if type exists
            var typ_class = self.device.plugins_classes.find(typ)
            if typ_class != nil
              var config = {'url': url, 'type': typ}
              typ_class.ui_string_to_conf(typ_class, config, arg)
              # check if configuration is already present
              var duplicate = false
              for c: self.device.plugins_config   # iterate on values, not on keys()
                # tasmota.log(string.format("MTR: map_compare '%s' ?= '%s' -> %s", str(c), str(config), str(self.equal_map(c,config))), 3)
                if self.equal_map(c,config)   duplicate = true  break   end
              end
              # not a duplicate, add it
              if !duplicate
                tasmota.log(string.format("MTR: remote add url='%s' type='%s' arg='%s'", url, typ, arg), 3)
                self.device.bridge_add_endpoint(typ, config)
              end
            end
          end
          idx += 1
          idx_str = str(idx)
        end
        #- and go back to Matter configuration -#
        webserver.redirect("/matterc?")

      else

      #---------------------------------------------------------------------#
      # Check if an endpoint needs to be deleted
      #---------------------------------------------------------------------#
        var ep_deleted
        for i:0..webserver.arg_size()-1
          var arg_name = webserver.arg_name(i)
          if string.find(arg_name, "del") == 0
            ep_deleted = int(arg_name[3..])
            break
          end
        end
        # check if we found an endpoint to be deleted
        if ep_deleted != nil
          tasmota.log(string.format("MTR: deleting endpoint %i", ep_deleted), 2)
          self.device.bridge_remove_endpoint(ep_deleted)
          webserver.redirect("/matterc?")
        end

      end

      if error
        webserver.content_start("Parameter error")           #- title of the web page -#
        webserver.content_send_style()                  #- send standard Tasmota styles -#
        webserver.content_send(string.format("<p style='width:340px;'><b>Error:</b>%s</p>", webserver.html_escape(error)))
        webserver.content_button(webserver.BUTTON_CONFIGURATION) #- button back to configuration page -#
        webserver.content_stop()                        #- end of web page -#
      end

    except .. as e, m
      tasmota.log(string.format("BRY: Exception> '%s' - %s", e, m), 2)
      #- display error page -#
      webserver.content_start("Parameter error")           #- title of the web page -#
      webserver.content_send_style()                  #- send standard Tasmota styles -#

      webserver.content_send(string.format("<p style='width:340px;'><b>Exception:</b><br>'%s'<br>%s</p>", e, m))

      webserver.content_button(webserver.BUTTON_CONFIGURATION) #- button back to configuration page -#
      webserver.content_stop()                        #- end of web page -#
    end
  end

  #######################################################################
  # Show bridge status
  #######################################################################
  def show_bridge_status()
    import webserver
    import string
    var bridge_plugin_by_host
    
    var idx = 0
    while idx < size(self.device.plugins)
      var plg = self.device.plugins[idx]

      if isinstance(plg, matter.Plugin_Bridge_HTTP)
        if bridge_plugin_by_host == nil     bridge_plugin_by_host = {}   end
        var host = plg.http_remote.addr

        if !bridge_plugin_by_host.contains(host)    bridge_plugin_by_host[host] = []    end
        bridge_plugin_by_host[host].push(plg)

      end
      idx += 1
    end

    if bridge_plugin_by_host == nil     return    end         # no remote device, abort

    # set specific styles
    webserver.content_send("<hr>")
    webserver.content_send("<table style='width:100%'>")
    webserver.content_send(matter._STYLESHEET)

    for host: bridge_plugin_by_host.keys()
      webserver.content_send(string.format("<tr class='ztdm htrm'><td><b>%s</b></td>", webserver.html_escape(host)))
      var http_remote = bridge_plugin_by_host[host][0].http_remote    # get the http_remote object from the first in list
      webserver.content_send(http_remote.web_last_seen())

      for plg: bridge_plugin_by_host[host]
        webserver.content_send("<tr class='htrm'><td colspan='2'>")
        plg.web_values()                                      # show values
        webserver.content_send("</td></tr>")
      end
    end


    webserver.content_send("</table><hr>")

  end

  #- display sensor value in the web UI -#
  def web_sensor()
    import webserver
    import string

    if self.matter_enabled()

      # mtc0 = close, mtc1 = open commissioning
      var fabrics_count = self.device.sessions.count_active_fabrics()
      if fabrics_count == 0
        webserver.content_send(string.format("<div style='text-align:right;font-size:11px;color:#aaa;padding:0px;'>%s</div>", "Matter: No active association"))
      else
        var plural = fabrics_count > 1
        webserver.content_send(string.format("<div style='text-align:right;font-size:11px;color:#aaa;padding:0px;'>%s</div>", "Matter: " + str(fabrics_count) + " active association" + (plural ? "s" : "")))
      end

      self.show_bridge_status()

      if self.device.is_root_commissioning_open()
        self.show_commissioning_info()
      end

    end
  end

  def web_get_arg()
    import webserver
    if   webserver.has_arg("mtc0")    # Close Commissioning
      self.device.stop_basic_commissioning()
    elif webserver.has_arg("mtc1")    # Open Commissioning
      self.device.start_root_basic_commissioning()
    end
  end

  #- ---------------------------------------------------------------------- -#
  # respond to web_add_handler() event to register web listeners
  #- ---------------------------------------------------------------------- -#
  #- this is called at Tasmota start-up, as soon as Wifi/Eth is up and web server running -#
  def web_add_handler()
    import webserver
    #- we need to register a closure, not just a function, that captures the current instance -#
    webserver.on("/matterc", / -> self.page_part_mgr(), webserver.HTTP_GET)
    webserver.on("/matterc", / -> self.page_part_ctl(), webserver.HTTP_POST)
    webserver.on("/mattera", / -> self.page_part_mgr_adv(), webserver.HTTP_GET)   # advanced
    webserver.on("/matteradd", / -> self.page_part_mgr_add(), webserver.HTTP_GET)   # add endpoint
  end
end
matter.UI = Matter_UI
