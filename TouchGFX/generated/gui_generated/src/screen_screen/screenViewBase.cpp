/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/screen_screen/screenViewBase.hpp>
#include <touchgfx/Color.hpp>
#include <BitmapDatabase.hpp>

screenViewBase::screenViewBase() :
    buttonCallback(this, &screenViewBase::buttonCallbackHandler)
{

    __background.setPosition(0, 0, 240, 320);
    __background.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));

    box1.setPosition(0, 0, 240, 320);
    box1.setColor(touchgfx::Color::getColorFromRGB(31, 82, 22));

    button1.setXY(35, 130);
    button1.setBitmaps(touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_EDGE_SMALL_ID), touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_EDGE_SMALL_PRESSED_ID));
    button1.setAction(buttonCallback);

    add(__background);
    add(box1);
    add(button1);
}

void screenViewBase::setupScreen()
{

}

void screenViewBase::buttonCallbackHandler(const touchgfx::AbstractButton& src)
{
    if (&src == &button1)
    {
        //Interaction1
        //When button1 clicked change screen to Screen1
        //Go to Screen1 with no screen transition
        application().gotoScreen1ScreenNoTransition();
    }
}
