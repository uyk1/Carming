import {createNativeStackNavigator} from '@react-navigation/native-stack';
import ReceiptScreen from '../screens/ReceiptScreen';
import ReviewWirteScreen from '../screens/ReviewWirteScreen';

const Stack = createNativeStackNavigator();

function L4_JourneyEndStackNavigator() {
  return (
    <Stack.Navigator
      initialRouteName="Receipt"
      screenOptions={{
        headerShown: false,
      }}>
      <Stack.Screen name="Receipt" component={ReceiptScreen} />
      <Stack.Screen name="Review" component={ReviewWirteScreen} />
    </Stack.Navigator>
  );
}

export default L4_JourneyEndStackNavigator;
