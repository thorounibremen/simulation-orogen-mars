require 'orocos'
require 'readline'
include Orocos

Orocos.initialize

#Orocos.run 'mars::Task' => 'mars', :cmdline_args => {'no-qtapp' => '0'} do #, "valgrind" => false do
Orocos.run 'mars::Task' => 'mars' do

    mars = TaskContext.get 'mars'
#    mars.controller_port = 1600
#    mars.enable_gui = 1

#    option_t = Orocos.registry.get 'simulation/Option'
#    option = option_t.new
#    option.name = "-c"
#    option.parameter = "1601"
#
#    raw_options = mars.raw_options
#    raw_options << option
#
#    mars.raw_options = raw_options

    mars.configure
    mars.start
   
#    sleep 50
#
#    STDOUT.puts "Restartign mars"
#    mars.stop
#    mars.cleanup
#    mars.configure
#    mars.start

    Readline::readline("Press ENTER to quit") do
    end

end 
