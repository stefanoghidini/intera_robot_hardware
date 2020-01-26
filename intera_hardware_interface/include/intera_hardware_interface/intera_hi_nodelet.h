#pragma once

# include <controller_manager/controller_manager.h>
# include <nodelet/nodelet.h>
# include <thread>
# include <intera_hardware_interface/intera_hardware_interface.h>
# include <itia_basic_hardware_interface/basic_hi_nodelet.h>
namespace itia
{
  namespace control
  {
    
    class InteraHwIfaceNodelet : public BasicHwIfaceNodelet
    {
    public:
      virtual void onInit();
      
    protected:
    };
    
    
    
  }
}
