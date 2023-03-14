import { createDrawerNavigator } from '@react-navigation/drawer';
import { HomeScreen } from '../screens';

const Drawer = createDrawerNavigator();

function DrawerNavigator() {
    return (
        <Drawer.Navigator 
            initialRouteName="Home"
            screenOptions={{
                drawerPosition: 'right',
                headerShown: false
            }}
        >
            <Drawer.Screen name="Home" component={HomeScreen} />
        </Drawer.Navigator>
    )
}

export default DrawerNavigator;