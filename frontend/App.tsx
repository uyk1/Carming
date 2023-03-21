/**
 * Sample React Native App
 * https://github.com/facebook/react-native
 *
 * @format
 */

import {NavigationContainer} from '@react-navigation/native';
import {useState} from 'react';
import L1_RootStackNavigator from './src/navigations/L1_RootStackNavigator';
import LaunchScreen from './src/screens/LaunchScreen';
import {MD3LightTheme, Provider as PaperProvider} from 'react-native-paper';

const theme = {
  ...MD3LightTheme,
  colors: {
    ...MD3LightTheme.colors,
    primary: '#7173C9',
    secondary: '#DF94C2',
    tertiary: '#FFBDC1',
    quaternary: '#141060',
    surface: '#8398D1',
    accent: '#70558E',
    neutral: '#D9D9D9',
  },
};

function App(): JSX.Element {
  const [isLoaded, setIsLoaded] = useState(false);

  setTimeout(() => {
    setIsLoaded(true);
  }, 2000);

  return (
    <PaperProvider theme={theme}>
      {isLoaded ? (
        <NavigationContainer>
          <L1_RootStackNavigator />
        </NavigationContainer>
      ) : (
        <LaunchScreen />
      )}
    </PaperProvider>
  );
}

export default App;
