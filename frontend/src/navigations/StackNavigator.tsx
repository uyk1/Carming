import { createNativeStackNavigator } from '@react-navigation/native-stack';
import { DrawerNavigator } from '../navigations';

const Stack = createNativeStackNavigator();

function StackNavigator() {
    return (
        <Stack.Navigator
            screenOptions={{
                headerShown: false,
            }}
        >
            <Stack.Screen name="DrawerNavigator" component={DrawerNavigator} />
        </Stack.Navigator>
    )
}

export default StackNavigator;